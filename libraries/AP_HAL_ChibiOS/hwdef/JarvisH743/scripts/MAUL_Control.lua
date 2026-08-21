-- =============================================================================
-- MAUL_Control.lua — Can-Am/BRP UGV: CAN decode + brake/gear control + throttle
-- gate, all in one script (UGV_Maul)
-- =============================================================================
-- IMPORTANT: this used to be two files (MAUL_CanDecode.lua + MAUL_Control.lua)
-- sharing data through a global table. That does NOT work in ArduPilot's Lua
-- sandbox: every loaded script gets its OWN private globals table
-- (lua_scripts.cpp: create_sandbox() + lua_setupvalue(L, -2, 1) rebinds each
-- chunk's _ENV to a fresh table), so a "global" set in one script is simply
-- invisible to another. Merged into one script so the CAN state is just a
-- normal local shared within this file.
--
-- Ackermann rover. Steering (rack) AND gas (pedal actuator) are both driven
-- NATIVELY by ArduPilot's own rover control (k_steering / k_throttle) — this
-- keeps ATC_SPEED cruise control, MOT_SLEWRATE, RC/GCS failsafe and AUTO/GUIDED
-- throttle demand all working exactly as ArduPilot intends. See
-- debug/MAUL_UGV_README.md for how the RC1 throttle channel is calibrated
-- (RC1_TRIM/RC1_DZ, symmetric idle zone e.g. 1470-1530) so the native
-- pipeline already outputs 0% for any stick position inside that zone (and
-- below it, via the SERVO2_MIN=SERVO2_TRIM clamp), ramping to 100% above it.
--
-- This script does NOT compute the gas PWM. It only:
--   1) forces SERVO2 (gas) to idle when the CAN-confirmed gear is not R/L/H
--      (the accelerator must physically do nothing in P/N or while the gear
--      state is unknown/stale) — same technique as neutralize_throttle() in
--      JRVS_MamontUGV.lua: override only to force a safe state, otherwise
--      leave the channel alone so ArduPilot's native output shows through.
--   2) drives the brake actuator proportionally from the raw CH1 stick
--      position below the idle zone (brake is NOT part of ArduPilot's motor
--      library — plain scripting output).
--   3) drives the P/N/R/L/H gear-selector actuator from CH_GEAR, gated by a
--      safety interlock: a gear change is only applied while CH1 is in the
--      idle zone AND CAN-reported speed (0x231) is ~0.
--
-- CH_THROTTLE (RC_OVERRIDE): raw PWM read directly here for brake + interlock
--                             decisions ONLY. The actual gas PWM is computed
--                             by ArduPilot's native throttle mixer from the
--                             same physical channel (RCMAP_THROTTLE=1).
-- CH_GEAR     (RC_OVERRIDE): 5 zones -> P / N / R / L / H target
--
-- CAN: passive listener only (never writes to the bus), decodes the Can-Am
-- broadcast frames documented in debug/canam_can_map.md. Requires one CAN
-- port set to CAN_Dx_PROTOCOL=10 (Scripting), bitrate 500000.
--
-- Bring-up logging (see debug/MAUL_UGV_README.md): a "first frame seen"
-- message logs once per known CAN ID the first time it's actually observed
-- on the bus, and a "no CAN frames received yet" warning repeats every 5s
-- until something arrives -- both always on. MAUL_CANDBG=1 (default) also
-- logs a periodic one-line summary of the decoded values every
-- MAUL_CANDBG_MS; set to 0 once you're done bench-testing.
-- =============================================================================

local SEV_INFO = 6
local SEV_WARN = 4
local function log(sev, msg) gcs:send_text(sev, "MAUL: " .. msg) end

-- ---------------------------------------------------------------------------
-- RC CHANNELS
-- ---------------------------------------------------------------------------
local CH_THROTTLE = 1
local CH_GEAR     = 6

-- ---------------------------------------------------------------------------
-- SERVO OUTPUTS (0-based channel index)
--   GAS_CHAN   -> SERVO2, SERVO2_FUNCTION=70 (k_throttle, native AP output).
--                 This script only ever forces it to idle; it never computes
--                 a "drive" PWM for it.
--   GEAR_CHAN  -> SERVO3, SERVO3_FUNCTION=94 (k_scripting1)
--   BRAKE_CHAN -> SERVO4, SERVO4_FUNCTION=95 (k_scripting2)
-- ---------------------------------------------------------------------------
local STEER_CHAN = 0  -- SERVO1
local GAS_CHAN   = 1  -- SERVO2
local GEAR_CHAN  = 2  -- SERVO3
local BRAKE_CHAN = 3  -- SERVO4
local OUT_TIMEOUT_MS = 200

local STEER_FUNC = 26 -- k_steering, for SRV_Channels:get_output_pwm()

local GAS_IDLE_PWM = 1500

-- ---------------------------------------------------------------------------
-- CUSTOM PARAMETERS (table key 73, prefix MAUL_)
-- ---------------------------------------------------------------------------
local PARAM_TABLE_KEY    = 73
local PARAM_TABLE_PREFIX = "MAUL_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 19), "MAUL: add_table failed")

local function add_param(idx, name, default)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default), "MAUL: add_param " .. name)
    local p = Parameter()
    assert(p:init(PARAM_TABLE_PREFIX .. name), "MAUL: init " .. name)
    return p
end

-- Gear-selector actuator PWM per position. UNCALIBRATED: default is a neutral
-- placeholder (1500) for every position until measured on the real actuator.
local P_P_SVO    = add_param(1,  'P_SVO',      1500)
local P_N_SVO    = add_param(2,  'N_SVO',      1500)
local P_R_SVO    = add_param(3,  'R_SVO',      1500)
local P_L_SVO    = add_param(4,  'L_SVO',      1500)
local P_H_SVO    = add_param(5,  'H_SVO',      1500)
local P_BRK_MIN  = add_param(6,  'BRK_MIN',    1000)  -- brake released
local P_BRK_MAX  = add_param(7,  'BRK_MAX',    2000)  -- brake fully squeezed
local P_THR_DZLO = add_param(8,  'THR_DZ_LO',  1470)  -- idle zone low bound (raw CH1)
local P_THR_DZHI = add_param(9,  'THR_DZ_HI',  1530)  -- idle zone high bound (raw CH1) -- keep in sync with RC1_TRIM +/- RC1_DZ!
local P_GEAR_SPD = add_param(10, 'GEAR_SPD_MX',  0.3) -- max CAN speed (km/h) to allow gear change
local P_CAN_TOMS = add_param(11, 'CAN_TIMEOUT', 200)  -- CAN feedback staleness timeout (ms)

-- Speed-based steering attenuation. Disabled by default (0) until bench/road
-- tested -- see debug/MAUL_UGV_README.md.
local P_STR_ATT_EN  = add_param(12, 'STR_ATT_EN',   0)   -- 0=disabled, 1=enabled
local P_STR_SPD_MAX = add_param(13, 'STR_SPD_MAX', 50)   -- km/h at which gain reaches STR_MIN_GAIN
local P_STR_MINGAIN = add_param(14, 'STR_MIN_GAIN', 0.3) -- steering gain (0-1) at/above STR_SPD_MAX

-- Gear-selector calibration sweep assist. Set MAUL_CAL_RUN=1 (bench only,
-- gearbox open, see debug/MAUL_UGV_README.md) to slowly sweep SERVO3 across
-- its full range while recording, per gear, the PWM window in which CAN
-- 0x309 reports that gear -- then log a suggested MAUL_*_SVO midpoint for
-- each. Auto-clears itself back to 0 when the sweep completes.
local P_CAL_RUN     = add_param(15, 'CAL_RUN',      0)   -- 0=idle, 1=start/running sweep
local P_CAL_STEP    = add_param(16, 'CAL_STEP',     5)   -- pwm step per tick
local P_CAL_STEPMS  = add_param(17, 'CAL_STEP_MS', 200)  -- ms between steps (let mechanism settle)

-- Bring-up CAN debug logging (see header comment).
local P_CANDBG     = add_param(18, 'CANDBG',     1)     -- 0=off, 1=periodic decoded summary in GCS messages
local P_CANDBG_MS  = add_param(19, 'CANDBG_MS', 2000)   -- ms between summary logs when CANDBG=1

local cfg = {}
local function refresh_cfg()
    cfg.P_SVO      = P_P_SVO:get()
    cfg.N_SVO      = P_N_SVO:get()
    cfg.R_SVO      = P_R_SVO:get()
    cfg.L_SVO      = P_L_SVO:get()
    cfg.H_SVO      = P_H_SVO:get()
    cfg.BRK_MIN    = P_BRK_MIN:get()
    cfg.BRK_MAX    = P_BRK_MAX:get()
    cfg.THR_DZ_LO  = P_THR_DZLO:get()
    cfg.THR_DZ_HI  = P_THR_DZHI:get()
    cfg.GEAR_SPD_MX = P_GEAR_SPD:get()
    cfg.CAN_TIMEOUT = P_CAN_TOMS:get()
    cfg.STR_ATT_EN  = P_STR_ATT_EN:get()
    cfg.STR_SPD_MAX = P_STR_SPD_MAX:get()
    cfg.STR_MIN_GAIN = P_STR_MINGAIN:get()
    cfg.CAL_RUN     = P_CAL_RUN:get()
    cfg.CAL_STEP    = P_CAL_STEP:get()
    cfg.CAL_STEP_MS = P_CAL_STEPMS:get()
    cfg.CANDBG      = P_CANDBG:get()
    cfg.CANDBG_MS   = P_CANDBG_MS:get()
end

local function gear_pwm(gear)
    if gear == "P" then return cfg.P_SVO
    elseif gear == "N" then return cfg.N_SVO
    elseif gear == "R" then return cfg.R_SVO
    elseif gear == "L" then return cfg.L_SVO
    elseif gear == "H" then return cfg.H_SVO
    else return cfg.P_SVO end
end

-- ---------------------------------------------------------------------------
-- GEAR-SELECTOR CHANNEL ZONES (provisional 5-way split, adjust to match the
-- actual switch/pot RC_OVERRIDE spread once bench-tested)
-- ---------------------------------------------------------------------------
local function target_gear_from_pwm(v)
    if v < 1180 then return "P"
    elseif v < 1360 then return "N"
    elseif v < 1540 then return "R"
    elseif v < 1720 then return "L"
    else return "H" end
end

-- ---------------------------------------------------------------------------
-- HELPERS
-- ---------------------------------------------------------------------------
local function rc_pwm(chan, default)
    local v = rc:get_pwm(chan)
    if not v or v < 900 then return default end
    return v
end

local function map(x, in_lo, in_hi, out_lo, out_hi)
    local t = (x - in_lo) / (in_hi - in_lo)
    if t < 0 then t = 0 elseif t > 1 then t = 1 end
    return out_lo + t * (out_hi - out_lo)
end

-- Steering gain at a given speed: 1.0 at 0 km/h, linearly down to
-- cfg.STR_MIN_GAIN at/above cfg.STR_SPD_MAX.
local function steering_gain(speed_kmh)
    if cfg.STR_SPD_MAX <= 0 then return cfg.STR_MIN_GAIN end
    local t = speed_kmh / cfg.STR_SPD_MAX
    if t < 0 then t = 0 elseif t > 1 then t = 1 end
    return 1.0 + t * (cfg.STR_MIN_GAIN - 1.0)
end

-- =============================================================================
-- CAN DECODE (was MAUL_CanDecode.lua) — passive Can-Am/BRP bus decoder.
-- `can_state` is a plain local, read directly by the control logic below in
-- the SAME script -- no cross-script sharing needed or attempted.
-- =============================================================================
local can_driver = CAN:get_device(25)
if not can_driver then
    log(SEV_WARN, "no scripting CAN driver found (check CAN_Dx_PROTOCOL=10)")
end

local can_state = {
    pedal_pct      = 0,
    pedal_pressed  = false,
    shaft_locked   = false,
    ts_103         = 0,

    gear           = "INVALID",
    gear_valid     = false,
    ts_309         = 0,

    speed_kmh      = 0,
    ts_231         = 0,

    rpm            = 0,
    engine_load    = 0,
    ts_102         = 0,

    drive_4wd      = false,
    ts_400         = 0,

    fuel_pct         = 0,
    fuel_sensor_open = false,
    ts_530           = 0,

    fuel_fault     = false,
    batt_voltage   = 0,
    ts_342         = 0,
}

local ID_PEDAL      = uint32_t(0x103)
local ID_GEAR       = uint32_t(0x309)
local ID_DRIVEMODE  = uint32_t(0x400)
local ID_SHAFTLOCK2 = uint32_t(0x121)
local ID_RPM        = uint32_t(0x102)
local ID_SPEED      = uint32_t(0x231)
local ID_FUEL       = uint32_t(0x530)
local ID_FUELFAULT  = uint32_t(0x342)

local GEAR_MAP = {
    [0xC0] = "P",
    [0x80] = "N",
    [0x40] = "R",
    [0x20] = "H",
    [0x10] = "L",
}

-- checksum used by 0x103 / 0x309 / 0x121: XOR of bytes 0-6 must equal byte 7
local function checksum_ok(frame)
    local x = 0
    for i = 0, 6 do
        x = x ~ frame:data(i)
    end
    return x == frame:data(7)
end

local function handle_pedal(frame, now)
    if not checksum_ok(frame) then return end
    can_state.pedal_pct     = frame:data(0) * 100.0 / 254.0
    can_state.pedal_pressed = (frame:data(3) & 0x40) ~= 0
    can_state.shaft_locked  = (frame:data(3) & 0x01) ~= 0
    can_state.ts_103        = now
end

local function handle_gear(frame, now)
    if not checksum_ok(frame) then return end
    local b0, b1 = frame:data(0), frame:data(1)
    local gear = GEAR_MAP[b0]
    can_state.gear_valid = (b0 == b1) and (gear ~= nil)
    can_state.gear       = can_state.gear_valid and gear or "INVALID"
    can_state.ts_309     = now
end

local function handle_shaftlock2(frame, now)
    if not checksum_ok(frame) then return end
    -- duplicate of the 0x103 lock bit, kept only as a cross-check / future use
    can_state.shaft_locked = (frame:data(0) & 0x01) ~= 0
    can_state.ts_103       = now
end

local function handle_drivemode(frame, now)
    can_state.drive_4wd = (frame:data(5) & 0x10) ~= 0
    can_state.ts_400    = now
end

local function handle_rpm(frame, now)
    can_state.rpm         = (frame:data(2) << 8) | frame:data(3)
    can_state.engine_load = frame:data(5)
    can_state.ts_102      = now
end

local function handle_speed(frame, now)
    can_state.speed_kmh = ((frame:data(0) << 8) | frame:data(1)) / 10.0
    can_state.ts_231    = now
end

local function handle_fuel(frame, now)
    local raw = frame:data(4)
    can_state.fuel_sensor_open = (raw == 0x7F)
    if raw ~= 0x7F then
        can_state.fuel_pct = raw
    end
    can_state.ts_530 = now
end

local function handle_fuelfault(frame, now)
    can_state.fuel_fault   = not (frame:data(2) == 0x99 and frame:data(3) == 0x99)
    can_state.batt_voltage = frame:data(4) * 0.1
    can_state.ts_342       = now
end

-- Bring-up debug bookkeeping
local KNOWN_IDS = {
    { id = 0x103, name = "pedal(0x103)" },
    { id = 0x309, name = "gear(0x309)" },
    { id = 0x121, name = "shaftlock2(0x121)" },
    { id = 0x400, name = "drivemode(0x400)" },
    { id = 0x102, name = "rpm(0x102)" },
    { id = 0x231, name = "speed(0x231)" },
    { id = 0x530, name = "fuel(0x530)" },
    { id = 0x342, name = "fuelfault(0x342)" },
}
local can_seen_ids     = {}
local can_unseen_count = #KNOWN_IDS

local can_total_frames    = 0
local can_last_summary_ms = 0
local can_last_nodata_ms  = 0

local function can_note_first_seen(frame)
    if can_unseen_count == 0 then return end
    local id_num = frame:id():toint()
    for _, k in ipairs(KNOWN_IDS) do
        if k.id == id_num and not can_seen_ids[k.id] then
            can_seen_ids[k.id] = true
            can_unseen_count = can_unseen_count - 1
            log(SEV_INFO, "CAN first frame seen: " .. k.name)
            break
        end
    end
end

-- Drains all pending CAN frames and refreshes bring-up logging. No-op if the
-- scripting CAN driver never came up.
local function can_update(now)
    if not can_driver then return end

    while true do
        local frame = can_driver:read_frame()
        if not frame then break end

        can_total_frames = can_total_frames + 1
        can_note_first_seen(frame)

        local id = uint32_t(frame:id())
        if id == ID_PEDAL then
            handle_pedal(frame, now)
        elseif id == ID_GEAR then
            handle_gear(frame, now)
        elseif id == ID_SHAFTLOCK2 then
            handle_shaftlock2(frame, now)
        elseif id == ID_DRIVEMODE then
            handle_drivemode(frame, now)
        elseif id == ID_RPM then
            handle_rpm(frame, now)
        elseif id == ID_SPEED then
            handle_speed(frame, now)
        elseif id == ID_FUEL then
            handle_fuel(frame, now)
        elseif id == ID_FUELFAULT then
            handle_fuelfault(frame, now)
        end
    end

    if can_total_frames == 0 then
        if now - can_last_nodata_ms > 5000 then
            log(SEV_WARN, "no CAN frames received yet - check wiring/CAN_Dx_PROTOCOL/bitrate")
            can_last_nodata_ms = now
        end
    elseif cfg.CANDBG ~= 0 then
        if now - can_last_summary_ms > cfg.CANDBG_MS then
            can_last_summary_ms = now
            log(SEV_INFO, string.format(
                "CAN frames=%d gear=%s(%s) spd=%.1f rpm=%d pedal=%.0f%% batt=%.1fV",
                can_total_frames,
                can_state.gear,
                can_state.gear_valid and "ok" or "inv",
                can_state.speed_kmh,
                can_state.rpm,
                can_state.pedal_pct,
                can_state.batt_voltage))
        end
    end
end

-- ---------------------------------------------------------------------------
-- STATE
-- ---------------------------------------------------------------------------
local target_gear         = "P"
local commanded_gear      = "P"     -- last gear actually applied to the actuator
local commanded_gear_pwm  = 1500    -- set from cfg.P_SVO on init
local last_warn_ms        = 0
local steer_trim_pwm      = 1500    -- read from SERVO1_TRIM on init

local function warn_throttled(msg)
    local now = millis()
    if now - last_warn_ms > 2000 then
        log(SEV_WARN, msg)
        last_warn_ms = now
    end
end

-- ---------------------------------------------------------------------------
-- GEAR CALIBRATION SWEEP (bench-only assist, see debug/MAUL_UGV_README.md)
-- ---------------------------------------------------------------------------
local cal_active       = false
local cal_pwm           = 1000
local cal_last_step_ms  = 0
local cal_windows       = {}

local function cal_reset(now)
    cal_pwm          = 1000
    cal_last_step_ms = now
    cal_windows      = { P = {}, N = {}, R = {}, L = {}, H = {} }
    log(SEV_INFO, "Gear cal: sweep started (bench only! gearbox must be safe to move)")
end

local function cal_finish()
    log(SEV_INFO, "Gear cal: sweep complete")
    for _, g in ipairs({ "P", "N", "R", "L", "H" }) do
        local w = cal_windows[g]
        if w.first then
            local mid = math.floor((w.first + w.last) / 2)
            log(SEV_INFO, string.format("Gear cal: %s seen %d-%d, suggest %s_SVO=%d", g, w.first, w.last, g, mid))
        else
            log(SEV_WARN, string.format("Gear cal: %s NOT seen during sweep", g))
        end
    end
    P_CAL_RUN:set_and_save(0)
    cal_active = false
end

-- Drives GEAR_CHAN itself. Returns nothing -- caller just needs to skip its
-- own GEAR_CHAN write while this is active.
local function calibration_tick(now)
    if not cal_active then
        cal_reset(now)
        cal_active = true
    end

    if now - cal_last_step_ms >= cfg.CAL_STEP_MS then
        cal_last_step_ms = now

        if can_state.gear_valid and (now - can_state.ts_309 < cfg.CAN_TIMEOUT) then
            local w = cal_windows[can_state.gear]
            if w then
                if not w.first then w.first = cal_pwm end
                w.last = cal_pwm
            end
        end

        cal_pwm = cal_pwm + cfg.CAL_STEP
        if cal_pwm > 2000 then
            cal_finish()
            return
        end
    end

    SRV_Channels:set_output_pwm_chan_timeout(GEAR_CHAN, math.floor(cal_pwm), OUT_TIMEOUT_MS)
end

-- ---------------------------------------------------------------------------
-- MAIN LOOP
-- ---------------------------------------------------------------------------
local function update()
    refresh_cfg()
    local now = millis()

    can_update(now)

    if not rc:has_valid_input() then
        -- fail safe: force gas idle (on top of ArduPilot's own RC failsafe),
        -- full brake, hold last commanded gear
        SRV_Channels:set_output_pwm_chan_timeout(GAS_CHAN,   GAS_IDLE_PWM, OUT_TIMEOUT_MS)
        SRV_Channels:set_output_pwm_chan_timeout(BRAKE_CHAN, cfg.BRK_MAX,  OUT_TIMEOUT_MS)
        SRV_Channels:set_output_pwm_chan_timeout(GEAR_CHAN,  commanded_gear_pwm, OUT_TIMEOUT_MS)
        return update, 10
    end

    local v1    = rc_pwm(CH_THROTTLE, cfg.THR_DZ_LO)  -- default: idle
    local vgear = rc_pwm(CH_GEAR, 1000)                -- default: P zone

    target_gear = target_gear_from_pwm(vgear)

    local can_gear_fresh  = (now - can_state.ts_309 < cfg.CAN_TIMEOUT)
    local can_speed_fresh = (now - can_state.ts_231 < cfg.CAN_TIMEOUT)
    local confirmed_gear  = (can_gear_fresh and can_state.gear_valid) and can_state.gear or "UNKNOWN"
    local speed_ok         = can_speed_fresh and (can_state.speed_kmh < cfg.GEAR_SPD_MX)

    local thr_neutral = (v1 >= cfg.THR_DZ_LO) and (v1 <= cfg.THR_DZ_HI)
    local interlock_ok = thr_neutral and speed_ok

    if cfg.CAL_RUN ~= 0 then
        -- bench calibration sweep owns GEAR_CHAN this tick; normal gear
        -- command/interlock logic is skipped entirely.
        calibration_tick(now)
    else
        if cal_active then cal_active = false end -- param was reset to 0 mid-sweep

        -- gear command: only act on a NEW target while the interlock is satisfied
        if target_gear ~= commanded_gear then
            if interlock_ok then
                commanded_gear     = target_gear
                commanded_gear_pwm = gear_pwm(commanded_gear)
                log(SEV_INFO, string.format("Gear -> %s (pwm=%d)", commanded_gear, commanded_gear_pwm))
            else
                warn_throttled(string.format(
                    "Gear change to %s blocked (thr_neutral=%s speed_ok=%s)",
                    target_gear, tostring(thr_neutral), tostring(speed_ok)))
            end
        end
    end

    -- brake: proportional below the idle zone, released elsewhere. Purely a
    -- function of the raw stick, independent of ArduPilot's own RC1 scaling.
    local brake_pwm
    if v1 < cfg.THR_DZ_LO then
        brake_pwm = map(v1, cfg.THR_DZ_LO, 1000, cfg.BRK_MIN, cfg.BRK_MAX)
    else
        brake_pwm = cfg.BRK_MIN
    end

    -- gas: only ever an override to force idle. When allowed, this script
    -- does not touch SERVO2 at all -- ArduPilot's native throttle mixer
    -- (RC1_TRIM/RC1_DZ, see README) already produces 0 output for any stick
    -- position at/below the idle zone and scales up into gas above it.
    local gas_allowed = (confirmed_gear == "R") or (confirmed_gear == "L") or (confirmed_gear == "H")
    if not gas_allowed then
        SRV_Channels:set_output_pwm_chan_timeout(GAS_CHAN, GAS_IDLE_PWM, OUT_TIMEOUT_MS)
    end

    -- speed-based steering attenuation: scale ArduPilot's own native steering
    -- output around trim, never compute a steering value from scratch. If
    -- speed is unknown/stale, fail safe by assuming max speed (most
    -- attenuation) rather than assuming stopped (full authority).
    if cfg.STR_ATT_EN ~= 0 then
        local steer_native = SRV_Channels:get_output_pwm(STEER_FUNC)
        if steer_native then
            local speed_for_gain = can_speed_fresh and can_state.speed_kmh or cfg.STR_SPD_MAX
            local gain = steering_gain(speed_for_gain)
            local steer_pwm = steer_trim_pwm + (steer_native - steer_trim_pwm) * gain
            SRV_Channels:set_output_pwm_chan_timeout(STEER_CHAN, math.floor(steer_pwm), OUT_TIMEOUT_MS)
        end
    end

    SRV_Channels:set_output_pwm_chan_timeout(BRAKE_CHAN, math.floor(brake_pwm), OUT_TIMEOUT_MS)
    if cfg.CAL_RUN == 0 then
        SRV_Channels:set_output_pwm_chan_timeout(GEAR_CHAN, math.floor(commanded_gear_pwm), OUT_TIMEOUT_MS)
    end

    return update, 10
end

-- ---------------------------------------------------------------------------
-- INIT
-- ---------------------------------------------------------------------------
local function init()
    refresh_cfg()
    commanded_gear     = "P"
    commanded_gear_pwm = cfg.P_SVO
    steer_trim_pwm      = math.floor(param:get('SERVO1_TRIM') or 1500)
    log(SEV_INFO, "MAUL_Control ready (gear params UNCALIBRATED until measured)")
    return update, 500
end

return init()
