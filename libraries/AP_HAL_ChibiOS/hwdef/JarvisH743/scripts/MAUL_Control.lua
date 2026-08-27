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
--   1) forces SERVO2 (gas) to idle while a gear shift is in flight or the
--      own-timer failsafe is active -- same technique as
--      neutralize_throttle() in JRVS_MamontUGV.lua: override only to force a
--      safe state, otherwise leave the channel alone so ArduPilot's native
--      output shows through. Gas is otherwise allowed in ANY gear, including
--      P/N, so the engine can be revved to check RPM response without
--      engaging the gearbox.
--   2) drives the brake actuator from several independent demand sources
--      (manual stick, gearshift-wait auto-brake, handbrake, hill-hold
--      auto-apply, failsafe) and outputs the max of them, with a standby
--      timer that drops PWM to literal 0 (servo board sleep) once the
--      vehicle is confirmed stationary and no longer needs holding force.
--   3) drives the P/N/R/L/H gear-selector actuator from CH_GEAR through a
--      state machine: wait-for-safe-speed/rpm (with auto-brake assist) ->
--      command sent + throttle locked -> CAN-confirmed or timed out ->
--      (if target is P) settle pause -> handbrake engaged; plus a hard
--      protect-timeout that stops driving the actuator if it never settles
--      (stuck gearbox), so the servo doesn't keep pulling stall current.
--   4) engages/auto-releases the front diff-lock relay from CH_DIFLOCK, with
--      a hard 0/1-hot interlock (not allowed while in gear H) and a
--      countdown auto-disable, reported to the GCS as a NAMED_VALUE_FLOAT so
--      an external dashboard (Maul-Docker) can show it.
--
-- CH_THROTTLE (RC_OVERRIDE): raw PWM read directly here for brake + interlock
--                             decisions ONLY. The actual gas PWM is computed
--                             by ArduPilot's native throttle mixer from the
--                             same physical channel (RCMAP_THROTTLE=1).
-- CH_GEAR     (RC_OVERRIDE): 5 zones -> P / N / R / L / H target
-- CH_DIFLOCK  (RC_OVERRIDE): rising edge = operator requests front diff-lock
-- CH_BRAKE_MANUAL (RC_OVERRIDE): dedicated manual brake, proportional over
--                             its full range, independent of CH1/gear/CAN
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
local CH_THROTTLE     = 1
local CH_GEAR         = 6
local CH_DIFLOCK      = 7  -- rising edge = operator requests diff-lock engage;
                            -- CANDIDATE channel, confirm free on the real remote
                            -- mapping before relying on it (see debug/MAUL_UGV_README.md)
local CH_BRAKE_MANUAL = 4  -- dedicated manual brake override, full range
                            -- 1000=released..2000=fully clamped, independent
                            -- of CH1 -- CANDIDATE channel, confirm free

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

-- Front diff-lock cutoff relay (physically in series with the operator's
-- tumbler line -- relay OFF = line broken = lock request cannot reach the
-- vehicle regardless of switch position). 0-based instance, see
-- AP_Scripting relay:on()/relay:off() -- RELAY1_PIN_DEFAULT on this board.
local DIFLOCK_RELAY = 0
local DIFLOCK_ON_THRESH = 1700

-- Brake-stick "full down" tolerance for handbrake-hold detection (raw CH1 is
-- 1000 at the very bottom of travel).
local BRAKE_FULL_DOWN_PWM = 1010

-- ---------------------------------------------------------------------------
-- CUSTOM PARAMETERS (table key 73, prefix MAUL_)
-- ---------------------------------------------------------------------------
local PARAM_TABLE_KEY    = 73
local PARAM_TABLE_PREFIX = "MAUL_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 40), "MAUL: add_table failed")

local function add_param(idx, name, default)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default), "MAUL: add_param " .. name)
    local p = Parameter()
    assert(p:init(PARAM_TABLE_PREFIX .. name), "MAUL: init " .. name)
    return p
end

-- All Parameter() objects live in one table, not one `local` each -- this
-- board's Lua build caps the main chunk at 100 local variables
-- ("too many local variables (limit is 100) in main function"), and 37
-- separate P_XXX locals (plus everything else at file scope) blew past that.
-- Table FIELD assignments below are NOT `local` declarations, so they don't
-- count against the limit -- only the one `local P = {}` does.
local P = {}

-- Gear-selector actuator PWM per position. Calibrated on the real actuator.
P.P_SVO    = add_param(1,  'P_SVO',      1020)
P.N_SVO    = add_param(2,  'N_SVO',      1510)
P.R_SVO    = add_param(3,  'R_SVO',      1260)
P.L_SVO    = add_param(4,  'L_SVO',      1980)
P.H_SVO    = add_param(5,  'H_SVO',      1750)
P.BRK_MIN  = add_param(6,  'BRK_MIN',    1000)  -- brake released
P.BRK_MAX  = add_param(7,  'BRK_MAX',    2000)  -- brake fully squeezed
P.THR_DZLO = add_param(8,  'THR_DZ_LO',  1470)  -- idle zone low bound (raw CH1)
P.THR_DZHI = add_param(9,  'THR_DZ_HI',  1530)  -- idle zone high bound (raw CH1) -- keep in sync with RC1_TRIM +/- RC1_DZ!
P.GEAR_SPD = add_param(10, 'GEAR_SPD_MX',  4)   -- GearShift_SafeSpeed (km/h): below this AND GEAR_SAFRPM, a shift may proceed
P.CAN_TOMS = add_param(11, 'CAN_TIMEOUT', 200)  -- CAN feedback staleness timeout (ms)

-- Speed-based steering attenuation ("progressive steering" per client spec).
-- Disabled by default (0) until bench/road tested -- see debug/MAUL_UGV_README.md.
P.STR_ATT_EN  = add_param(12, 'STR_ATT_EN',   0)   -- 0=disabled, 1=enabled
P.STR_SPD_MAX = add_param(13, 'STR_SPD_MAX', 50)   -- km/h at which gain reaches STR_MIN_GAIN
P.STR_MINGAIN = add_param(14, 'STR_MIN_GAIN', 0.3) -- steering gain (0-1) at/above STR_SPD_MAX

-- Gear-selector calibration sweep assist. Set MAUL_CAL_RUN=1 (bench only,
-- gearbox open, see debug/MAUL_UGV_README.md) to slowly sweep SERVO3 across
-- its full range while recording, per gear, the PWM window in which CAN
-- 0x309 reports that gear -- then log a suggested MAUL_*_SVO midpoint for
-- each. Auto-clears itself back to 0 when the sweep completes.
P.CAL_RUN     = add_param(15, 'CAL_RUN',      0)   -- 0=idle, 1=start/running sweep
P.CAL_STEP    = add_param(16, 'CAL_STEP',     5)   -- pwm step per tick
P.CAL_STEPMS  = add_param(17, 'CAL_STEP_MS', 200)  -- ms between steps (let mechanism settle)

-- Bring-up CAN debug logging (see header comment).
P.CANDBG     = add_param(18, 'CANDBG',     1)     -- 0=off, 1=periodic decoded summary in GCS messages
P.CANDBG_MS  = add_param(19, 'CANDBG_MS', 2000)   -- ms between summary logs when CANDBG=1

-- --- Gearbox mode logic ---
P.GEAR_SAFRPM = add_param(20, 'GEAR_SAFRPM', 1900)  -- GearShift_SafeRPM
P.GEAR_MAXT   = add_param(21, 'GEAR_MAXT',   4500)  -- GearShift_MaxTime (ms): expected actuator travel time + 15%
P.GEAR_TOUT   = add_param(22, 'GEAR_TOUT',  15000)  -- GearShift_Timeout (ms), added on top of GEAR_MAXT before protect-cutoff

-- --- Brake mode logic ---
P.BRK_AAPFRC  = add_param(23, 'BRK_AAPFRC',   40)   -- Brakes_AutoApply_Force (%)
P.BRK_HANDFRC = add_param(24, 'BRK_HANDFRC', 100)   -- Brakes_Handbrake_Force (%)
P.BRK_RELRPM  = add_param(25, 'BRK_RELRPM',  1950)  -- Brakes_Auto_Release_RPM
P.BRK_HOLDMS  = add_param(26, 'BRK_HOLDMS',  3000)  -- stick held full-down duration to engage handbrake (ms)
P.BRK_DBLPCT  = add_param(27, 'BRK_DBLPCT',    95)  -- stick travel %% for a "click" of the double-press release
P.BRK_DBLMS   = add_param(28, 'BRK_DBLMS',    600)  -- max gap between the two clicks (ms)
P.BRK_AADELAY = add_param(29, 'BRK_AADELAY', 5000)  -- Brakes_AutoApply_Delay (ms) -- PLACEHOLDER, client TZ gave no number, confirm
P.BRK_AASTEP  = add_param(30, 'BRK_AASTEP',    10)  -- hill-hold escalation step (%)
P.BRK_AAPRD   = add_param(31, 'BRK_AAPRD',   1000)  -- hill-hold escalation period (ms)
P.BRK_SBYMS   = add_param(32, 'BRK_SBYMS',  20000)  -- Brakes_Standby_Delay (ms) before PWM=0 sleep
P.BRK_GASPCT  = add_param(38, 'BRK_GASPCT',   10)   -- gas-stick %% (above idle) that also releases handbrake/hill-hold -- client-requested fallback for when CAN rpm isn't available

-- --- Failsafe (own timer, independent of native FS_TIMEOUT/FS_ACTION -- see
-- debug/MAUL_UGV_README.md; this one gates the scripting-owned brake/gear/gas
-- actuators that the native RC failsafe path doesn't touch) ---
P.FS_DLYMS   = add_param(33, 'FS_DLYMS',  800)   -- FS_Delay_time (ms)
P.FS_THRPWM  = add_param(34, 'FS_THRPWM', 1500)  -- FS_Throttle_PWM
P.FS_BRKFRC  = add_param(35, 'FS_BRKFRC',   70)  -- FS_Brake_force (%)

-- --- Steering: diff-lock max angle clamp ---
P.STR_DLMXANG = add_param(36, 'STR_DLMXANG', 100)  -- max PWM deviation from SERVO1_TRIM while diff-lock active -- UNCALIBRATED placeholder

-- --- Front diff-lock timer ---
P.DL_WORKT = add_param(37, 'DL_WORKT', 180)  -- DifLock_Work_Time (s)

local cfg = {}
local function refresh_cfg()
    cfg.P_SVO      = P.P_SVO:get()
    cfg.N_SVO      = P.N_SVO:get()
    cfg.R_SVO      = P.R_SVO:get()
    cfg.L_SVO      = P.L_SVO:get()
    cfg.H_SVO      = P.H_SVO:get()
    cfg.BRK_MIN    = P.BRK_MIN:get()
    cfg.BRK_MAX    = P.BRK_MAX:get()
    cfg.THR_DZ_LO  = P.THR_DZLO:get()
    cfg.THR_DZ_HI  = P.THR_DZHI:get()
    cfg.GEAR_SPD_MX = P.GEAR_SPD:get()
    cfg.CAN_TIMEOUT = P.CAN_TOMS:get()
    cfg.STR_ATT_EN  = P.STR_ATT_EN:get()
    cfg.STR_SPD_MAX = P.STR_SPD_MAX:get()
    cfg.STR_MIN_GAIN = P.STR_MINGAIN:get()
    cfg.CAL_RUN     = P.CAL_RUN:get()
    cfg.CAL_STEP    = P.CAL_STEP:get()
    cfg.CAL_STEP_MS = P.CAL_STEPMS:get()
    cfg.CANDBG      = P.CANDBG:get()
    cfg.CANDBG_MS   = P.CANDBG_MS:get()

    cfg.GEAR_SAFRPM = P.GEAR_SAFRPM:get()
    cfg.GEAR_MAXT   = P.GEAR_MAXT:get()
    cfg.GEAR_TOUT   = P.GEAR_TOUT:get()

    cfg.BRK_AAPFRC  = P.BRK_AAPFRC:get()
    cfg.BRK_HANDFRC = P.BRK_HANDFRC:get()
    cfg.BRK_RELRPM  = P.BRK_RELRPM:get()
    cfg.BRK_HOLDMS  = P.BRK_HOLDMS:get()
    cfg.BRK_DBLPCT  = P.BRK_DBLPCT:get()
    cfg.BRK_DBLMS   = P.BRK_DBLMS:get()
    cfg.BRK_AADELAY = P.BRK_AADELAY:get()
    cfg.BRK_AASTEP  = P.BRK_AASTEP:get()
    cfg.BRK_AAPRD   = P.BRK_AAPRD:get()
    cfg.BRK_SBYMS   = P.BRK_SBYMS:get()
    cfg.BRK_GASPCT  = P.BRK_GASPCT:get()

    cfg.FS_DLYMS   = P.FS_DLYMS:get()
    cfg.FS_THRPWM  = P.FS_THRPWM:get()
    cfg.FS_BRKFRC  = P.FS_BRKFRC:get()

    cfg.STR_DLMXANG = P.STR_DLMXANG:get()

    cfg.DL_WORKT = P.DL_WORKT:get()
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
-- actual switch/pot RC_OVERRIDE spread once bench-tested). Order confirmed
-- on the real selector 2026-08-22: P, R, N, H, L.
-- ---------------------------------------------------------------------------
local function target_gear_from_pwm(v)
    if v < 1180 then return "P"
    elseif v < 1360 then return "R"
    elseif v < 1540 then return "N"
    elseif v < 1720 then return "H"
    else return "L" end
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

local function clamp(x, lo, hi)
    if x < lo then return lo end
    if x > hi then return hi end
    return x
end

-- Percent (0-100) of full brake travel -> PWM, same scale the manual stick
-- mapping already uses.
local function pct_to_brake_pwm(pct)
    return cfg.BRK_MIN + (cfg.BRK_MAX - cfg.BRK_MIN) * (pct / 100.0)
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
    engine_temp_c  = 0,
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

-- one table, not 8 locals -- see the P table comment above re: the 100-local
-- main-chunk limit
local CAN_ID = {
    PEDAL      = uint32_t(0x103),
    GEAR       = uint32_t(0x309),
    DRIVEMODE  = uint32_t(0x400),
    SHAFTLOCK2 = uint32_t(0x121),
    RPM        = uint32_t(0x102),
    SPEED      = uint32_t(0x231),
    FUEL       = uint32_t(0x530),
    FUELFAULT  = uint32_t(0x342),
}

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

-- 0x102: RPM = (B0<<8|B1) x 0.25, engine temp = B3-60 degC -- fixed against
-- debug/canam_can_map_v2.md (spec-confirmed; v1's B2-3 raw-count read was
-- wrong, kept giving a plausible-looking but incorrect number).
local function handle_rpm(frame, now)
    can_state.rpm         = ((frame:data(0) << 8) | frame:data(1)) * 0.25
    can_state.engine_temp_c = frame:data(3) - 60
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
-- one table, not 5 locals -- see the P table comment re: the 100-local limit
local can_dbg = {
    seen_ids        = {},
    unseen_count    = #KNOWN_IDS,
    total_frames    = 0,
    last_summary_ms = 0,
    last_nodata_ms  = 0,
}

local function can_note_first_seen(frame)
    if can_dbg.unseen_count == 0 then return end
    local id_num = frame:id():toint()
    for _, k in ipairs(KNOWN_IDS) do
        if k.id == id_num and not can_dbg.seen_ids[k.id] then
            can_dbg.seen_ids[k.id] = true
            can_dbg.unseen_count = can_dbg.unseen_count - 1
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

        can_dbg.total_frames = can_dbg.total_frames + 1
        can_note_first_seen(frame)

        local id = uint32_t(frame:id())
        if id == CAN_ID.PEDAL then
            handle_pedal(frame, now)
        elseif id == CAN_ID.GEAR then
            handle_gear(frame, now)
        elseif id == CAN_ID.SHAFTLOCK2 then
            handle_shaftlock2(frame, now)
        elseif id == CAN_ID.DRIVEMODE then
            handle_drivemode(frame, now)
        elseif id == CAN_ID.RPM then
            handle_rpm(frame, now)
        elseif id == CAN_ID.SPEED then
            handle_speed(frame, now)
        elseif id == CAN_ID.FUEL then
            handle_fuel(frame, now)
        elseif id == CAN_ID.FUELFAULT then
            handle_fuelfault(frame, now)
        end
    end

    if can_dbg.total_frames == 0 then
        if now - can_dbg.last_nodata_ms > 5000 then
            log(SEV_WARN, "no CAN frames received yet - check wiring/CAN_Dx_PROTOCOL/bitrate")
            can_dbg.last_nodata_ms = now
        end
    elseif cfg.CANDBG ~= 0 then
        if now - can_dbg.last_summary_ms > cfg.CANDBG_MS then
            can_dbg.last_summary_ms = now
            log(SEV_INFO, string.format(
                "CAN frames=%d gear=%s(%s) spd=%.1f rpm=%.0f temp=%dC pedal=%.0f%% batt=%.1fV",
                can_dbg.total_frames,
                can_state.gear,
                can_state.gear_valid and "ok" or "inv",
                can_state.speed_kmh,
                can_state.rpm,
                can_state.engine_temp_c,
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

-- Gear-shift state machine (section A)
local gear_wait_safe          = false  -- waiting for safe speed/rpm before commanding the actuator
local gear_wait_target        = "P"
local gear_cmd_pending        = false  -- actuator commanded, not yet confirmed/timed out -- throttle locked
local gear_cmd_start_ms       = 0
local gear_park_wait          = false  -- gear settled on P, waiting extra GEAR_MAXT before handbrake
local gear_park_wait_start_ms = 0

-- Brakes (section B)
local handbrake_engaged     = false
local brake_hold_start_ms   = 0    -- 0 = stick not currently full-down
local brake_click_prev      = false
local brake_click1_ms       = 0    -- 0 = no pending first click of a double-press
local hillhold_active       = false
local hillhold_pwm_pct      = 0
local hillhold_last_step_ms = 0
local hillhold_timer_start_ms = 0
local hillhold_last_gear    = "UNKNOWN"
-- "Never" until the first real auto-clamp event sets a real deadline. Can't
-- use math.huge here: millis()/now is a uint32_t userdata, and comparing it
-- against a float coerces the float to uint32_t (lua_boxed_numerics.cpp,
-- coerce_to_uint32_t) which requires 0 <= v <= UINT32_MAX -- math.huge fails
-- that range check and throws "bad argument" instead of just being "big".
-- 0xFFFFFFFF (UINT32_MAX) IS in range and is ~49.7 days of uptime away, so
-- it's effectively "never" for this vehicle's purposes.
local standby_deadline_ms   = 0xFFFFFFFF
-- Latched, not recomputed fresh each tick: once asleep, only a confirmed-real
-- motion reading, a live manual/override press, or a fresh auto-clamp event
-- wakes it -- CAN merely going stale must NOT wake it (client-reported bug:
-- previously, losing CAN while already asleep made PWM "reappear" instantly,
-- because the old sleep_ok check required *continuously* fresh CAN showing
-- zero speed/rpm to justify staying at PWM=0).
local brake_asleep = false

-- Failsafe (section C) -- own timer, independent of native FS_TIMEOUT/FS_ACTION
local link_lost_start_ms = 0  -- 0 = link currently valid
local failsafe_active    = false

-- Front diff-lock (section E)
local difflock_active      = false
local difflock_deadline_ms = 0
local difflock_ch_prev     = false

-- Mode-status telemetry (MAUL_DL/MAUL_ST NAMED_VALUE_FLOAT, section F -- read
-- by Maul-Docker so the dashboard can show gearshift/brake/failsafe state,
-- not just raw CAN)
local status_tlm_last_ms = 0

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
    P.CAL_RUN:set_and_save(0)
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
-- GEAR STATE MACHINE HELPER (section A)
-- ---------------------------------------------------------------------------
local function start_gear_shift(gear, now)
    commanded_gear     = gear
    commanded_gear_pwm = gear_pwm(gear)
    gear_cmd_pending    = true
    gear_cmd_start_ms   = now
    log(SEV_INFO, string.format("Gear -> %s (pwm=%d)", gear, commanded_gear_pwm))
end

-- Releases handbrake/hill-hold, gated by "never while (believed to be) in P".
-- Caller passes effective_gear (CAN-confirmed, or commanded_gear fallback
-- when CAN is stale) so this still works without CAN.
local function release_brakes(gear_belief, reason)
    if gear_belief == "P" then return end
    if handbrake_engaged or hillhold_active then
        handbrake_engaged = false
        hillhold_active   = false
        hillhold_pwm_pct  = 0
        log(SEV_INFO, "Brakes released (" .. reason .. ")")
    end
end

-- ---------------------------------------------------------------------------
-- MAIN LOOP
-- ---------------------------------------------------------------------------
local function update()
    refresh_cfg()
    local now = millis()

    can_update(now)

    -- own failsafe timer -- separate from native FS_TIMEOUT/FS_ACTION, which
    -- still runs independently and covers what this script doesn't (mode
    -- changes etc). This one gates the scripting-owned gas/brake/gear outputs.
    if not rc:has_valid_input() then
        if link_lost_start_ms == 0 then link_lost_start_ms = now end
    else
        link_lost_start_ms = 0
        failsafe_active     = false
    end
    if link_lost_start_ms ~= 0 and (now - link_lost_start_ms >= cfg.FS_DLYMS) then
        failsafe_active = true
    end

    local v1        = rc_pwm(CH_THROTTLE, cfg.THR_DZ_LO)  -- default: idle
    local vgear      = rc_pwm(CH_GEAR, 1000)               -- default: P zone
    local vdiflock   = rc_pwm(CH_DIFLOCK, 1000)
    local v_brk_man  = rc_pwm(CH_BRAKE_MANUAL, 1000)       -- default: released

    target_gear = target_gear_from_pwm(vgear)

    local can_gear_fresh  = (now - can_state.ts_309 < cfg.CAN_TIMEOUT)
    local can_speed_fresh = (now - can_state.ts_231 < cfg.CAN_TIMEOUT)
    local can_rpm_fresh   = (now - can_state.ts_102 < cfg.CAN_TIMEOUT)
    local confirmed_gear  = (can_gear_fresh and can_state.gear_valid) and can_state.gear or "UNKNOWN"

    -- Best available belief about the current gear: CAN-confirmed when fresh
    -- (independent proof), else fall back to commanded_gear (what this
    -- script itself last told the actuator -- gear_cmd_pending being false
    -- by the time this matters means that command already either got
    -- CAN-confirmed or hit the GEAR_MAXT timeout). Used everywhere a "must
    -- not release brakes while actually in P" check needs to still work
    -- without CAN, per client spec -- see section B below.
    local effective_gear = can_gear_fresh and confirmed_gear or commanded_gear

    local thr_neutral = (v1 >= cfg.THR_DZ_LO) and (v1 <= cfg.THR_DZ_HI)

    -- CAN gives the precise speed/rpm check when it's actually alive. If
    -- either signal is stale, speed_ok/rpm_ok can't be evaluated against a
    -- real number -- default them to "ok" (not blocking) rather than "not
    -- ok forever", and gate on can_full_telemetry below to decide whether we
    -- trust that or fall back to a proxy. Without this, a dead/disconnected
    -- CAN tap (a reverse-engineered aftermarket tap, not vehicle-original --
    -- more likely to fail than the vehicle bus itself) would permanently
    -- wedge gear_wait_safe true and hold the gearshift-wait auto-brake on
    -- forever, i.e. the gearbox could never shift again until CAN came back.
    local can_full_telemetry = can_speed_fresh and can_rpm_fresh
    local speed_ok = (not can_speed_fresh) or (can_state.speed_kmh < cfg.GEAR_SPD_MX)
    local rpm_ok    = (not can_rpm_fresh) or (can_state.rpm < cfg.GEAR_SAFRPM)
    local safe_to_shift = speed_ok and rpm_ok

    -- =========================================================================
    -- A. GEARBOX STATE MACHINE
    -- =========================================================================
    if cfg.CAL_RUN ~= 0 then
        -- bench calibration sweep owns GEAR_CHAN this tick; normal gear
        -- command/interlock logic is skipped entirely.
        calibration_tick(now)
    else
        if cal_active then cal_active = false end -- param was reset to 0 mid-sweep

        if not failsafe_active then
            if target_gear ~= commanded_gear and not gear_cmd_pending and not gear_wait_safe then
                if can_full_telemetry then
                    -- normal path: precise speed/rpm gate, with auto-brake
                    -- assist (gear_wait_safe below) if not currently safe
                    if safe_to_shift then
                        start_gear_shift(target_gear, now)
                    else
                        gear_wait_safe   = true
                        gear_wait_target = target_gear
                        warn_throttled(string.format(
                            "Gear -> %s waiting for safe speed/rpm (spd_ok=%s rpm_ok=%s)",
                            target_gear, tostring(speed_ok), tostring(rpm_ok)))
                    end
                elseif thr_neutral then
                    -- CAN unavailable: no auto-brake (we have no real speed
                    -- to wait on -- braking blind would be presumptuous),
                    -- fall back to the one always-available proxy for "not
                    -- asking to accelerate": throttle stick in the idle zone.
                    start_gear_shift(target_gear, now)
                else
                    warn_throttled(string.format(
                        "Gear -> %s blocked: no CAN telemetry, release throttle to shift",
                        target_gear))
                end
            end

            if gear_wait_safe then
                if target_gear ~= gear_wait_target then
                    gear_wait_target = target_gear -- operator changed their mind while waiting
                end
                if can_full_telemetry then
                    if safe_to_shift then
                        gear_wait_safe = false
                        start_gear_shift(gear_wait_target, now)
                    end
                elseif thr_neutral then
                    -- CAN died mid-wait -- stop guessing at a speed we can no
                    -- longer measure, drop back to the throttle-neutral gate
                    gear_wait_safe = false
                    start_gear_shift(gear_wait_target, now)
                end
            end
        end

        if gear_cmd_pending then
            local shift_confirmed = can_gear_fresh and can_state.gear_valid and can_state.gear == commanded_gear
            if shift_confirmed or (now - gear_cmd_start_ms > cfg.GEAR_MAXT) then
                gear_cmd_pending = false
                if commanded_gear == "P" then
                    gear_park_wait          = true
                    gear_park_wait_start_ms = now
                end
            end
        end

        if gear_park_wait and (now - gear_park_wait_start_ms > cfg.GEAR_MAXT) then
            gear_park_wait = false
            if not handbrake_engaged then
                handbrake_engaged   = true
                standby_deadline_ms = now + cfg.BRK_SBYMS
                brake_asleep        = false
                log(SEV_INFO, "Parking: handbrake engaged")
            end
        end
    end

    -- =========================================================================
    -- B. BRAKES -- independent demand sources, output the max, standby sleep
    -- =========================================================================
    local manual_pwm
    local manual_pressed = v1 < cfg.THR_DZ_LO
    if manual_pressed then
        manual_pwm = map(v1, cfg.THR_DZ_LO, 1000, cfg.BRK_MIN, cfg.BRK_MAX)
    else
        manual_pwm = cfg.BRK_MIN
    end

    -- dedicated manual brake override (CH_BRAKE_MANUAL) -- independent of
    -- CH1/gear/CAN entirely, proportional across the channel's full range so
    -- it works whether it's wired to a switch (effectively on/off) or a
    -- lever. Always live, no latching/state-machine -- releases the instant
    -- the channel drops back to idle, same as the CH1 manual brake.
    local override_pressed = v_brk_man > 1010
    local override_pwm = override_pressed and map(v_brk_man, 1000, 2000, cfg.BRK_MIN, cfg.BRK_MAX) or 0

    -- handbrake engage: stick held full-down continuously for BRK_HOLDMS
    local full_down = v1 <= BRAKE_FULL_DOWN_PWM
    if full_down then
        if brake_hold_start_ms == 0 then brake_hold_start_ms = now end
        if not handbrake_engaged and (now - brake_hold_start_ms) >= cfg.BRK_HOLDMS then
            handbrake_engaged    = true
            standby_deadline_ms  = now + cfg.BRK_SBYMS
            brake_asleep         = false
            log(SEV_INFO, "Handbrake engaged (stick hold)")
        end
    else
        brake_hold_start_ms = 0
    end

    -- double-click release: two "stick crossed BRK_DBLPCT%% down" edges
    -- within BRK_DBLMS of each other
    local dbl_thresh_pwm = cfg.THR_DZ_LO - (cfg.THR_DZ_LO - 1000) * (cfg.BRK_DBLPCT / 100.0)
    local click_active   = v1 <= dbl_thresh_pwm
    if click_active and not brake_click_prev then
        if brake_click1_ms ~= 0 and (now - brake_click1_ms) <= cfg.BRK_DBLMS then
            release_brakes(effective_gear, "double-click")
            brake_click1_ms = 0
        else
            brake_click1_ms = now
        end
    end
    if brake_click1_ms ~= 0 and (now - brake_click1_ms) > cfg.BRK_DBLMS then
        brake_click1_ms = 0
    end
    brake_click_prev = click_active

    -- deliberate movement releases handbrake/hill-hold, and cancels a
    -- pending hill-hold auto-apply delay: rpm above the release threshold
    -- (needs CAN), OR gas stick pushed BRK_GASPCT%% above idle (client-
    -- requested fallback proxy for when CAN rpm isn't available -- also a
    -- valid, slightly earlier signal even when CAN is up, so it's not
    -- gated to the no-CAN case specifically).
    local gas_release_pwm = cfg.THR_DZ_HI + (2000 - cfg.THR_DZ_HI) * (cfg.BRK_GASPCT / 100.0)
    local gas_deliberate  = v1 >= gas_release_pwm
    local deliberate_rpm  = can_rpm_fresh and can_state.rpm >= cfg.BRK_RELRPM
    local deliberate_move = deliberate_rpm or gas_deliberate
    if deliberate_move then
        release_brakes(effective_gear, deliberate_rpm and "rpm" or "gas stick")
    end

    -- hill-hold: if no deliberate throttle within BRK_AADELAY of a (non-P)
    -- gear being confirmed, auto-apply and escalate while still rolling
    if confirmed_gear ~= hillhold_last_gear then
        hillhold_last_gear     = confirmed_gear
        hillhold_timer_start_ms = now
    end
    if deliberate_move then
        hillhold_timer_start_ms = now
    end
    if (not hillhold_active) and (not handbrake_engaged)
        and confirmed_gear ~= "P" and confirmed_gear ~= "UNKNOWN" and confirmed_gear ~= "INVALID"
        and (now - hillhold_timer_start_ms) >= cfg.BRK_AADELAY then
        hillhold_active       = true
        hillhold_pwm_pct      = cfg.BRK_AAPFRC
        hillhold_last_step_ms = now
        standby_deadline_ms   = now + cfg.BRK_SBYMS
        brake_asleep          = false
        log(SEV_WARN, "Auto-brake: no throttle after gear engage")
    end
    if hillhold_active and can_speed_fresh and can_state.speed_kmh > 0
        and (now - hillhold_last_step_ms) >= cfg.BRK_AAPRD then
        hillhold_pwm_pct      = math.min(hillhold_pwm_pct + cfg.BRK_AASTEP, 100)
        hillhold_last_step_ms = now
        standby_deadline_ms   = now + cfg.BRK_SBYMS
        brake_asleep          = false
    end

    -- gearshift-wait auto-brake (section A.1)
    local gearwait_pwm = gear_wait_safe and pct_to_brake_pwm(cfg.BRK_AAPFRC) or 0

    -- failsafe brake (section C) -- full force while moving, eased to
    -- AutoApply level once CAN confirms the vehicle actually stopped
    local failsafe_pwm = 0
    if failsafe_active then
        local stopped = can_speed_fresh and can_state.speed_kmh <= 0.1
        failsafe_pwm = pct_to_brake_pwm(stopped and cfg.BRK_AAPFRC or cfg.FS_BRKFRC)
    end

    local handbrake_src = handbrake_engaged and pct_to_brake_pwm(cfg.BRK_HANDFRC) or 0
    local hillhold_src   = hillhold_active and pct_to_brake_pwm(hillhold_pwm_pct) or 0

    local final_brake_pwm = manual_pwm
    if override_pwm  > final_brake_pwm then final_brake_pwm = override_pwm end
    if gearwait_pwm  > final_brake_pwm then final_brake_pwm = gearwait_pwm end
    if handbrake_src > final_brake_pwm then final_brake_pwm = handbrake_src end
    if hillhold_src  > final_brake_pwm then final_brake_pwm = hillhold_src end
    if failsafe_pwm  > final_brake_pwm then final_brake_pwm = failsafe_pwm end

    -- standby sleep: only for handbrake/hill-hold (per client spec), never
    -- while the gearshift-wait or failsafe sources are active, never while
    -- actually moving or above GearShift_SafeRPM (servo board wake latency),
    -- and never while the operator is actively holding either manual source.
    --
    -- Latched via brake_asleep (declared in STATE), not recomputed fresh
    -- every tick -- once asleep, only positive CAN-confirmed motion, a live
    -- manual/override press, gearshift-wait or failsafe wakes it; CAN merely
    -- going stale must NOT wake it (fixed bug 2026-08-23).
    --
    -- 2026-08-27 (client): the standby timer must ALSO fire with CAN
    -- absent/stale. "No fresh CAN speed/rpm" is therefore taken as "assume
    -- stopped" for the ENTER-sleep test (same convention as speed_ok/rpm_ok in
    -- the gear machine above) instead of blocking sleep forever. The WAKE test
    -- (definitely_moving, below) still needs positive fresh-CAN motion, so a
    -- stale bus never wakes an already-sleeping brake.
    local speed_is_zero = (not can_speed_fresh) or (can_state.speed_kmh <= 0.05)
    local rpm_is_low     = (not can_rpm_fresh)   or (can_state.rpm < cfg.GEAR_SAFRPM)

    local definitely_moving = (can_speed_fresh and can_state.speed_kmh > 0.05)
        or (can_rpm_fresh and can_state.rpm >= cfg.GEAR_SAFRPM)
    if definitely_moving or manual_pressed or override_pressed or gear_wait_safe or failsafe_active then
        brake_asleep = false
    end

    local can_enter_sleep = (not failsafe_active) and (not gear_wait_safe)
        and (not manual_pressed) and (not override_pressed) and speed_is_zero and rpm_is_low
        and (now > standby_deadline_ms)
    if can_enter_sleep then
        brake_asleep = true
    end

    -- Physical actuator is inverted vs. the logical BRK_MIN..BRK_MAX scale
    -- used everywhere above (BRK_MIN=released..BRK_MAX=squeezed): on the
    -- actual servo, 2000=released and 1000=fully squeezed. Mirror only at
    -- the very last step, so the internal demand/state-machine math above
    -- (and BRK_MIN/BRK_MAX's meaning) stays untouched; the 0 sleep sentinel
    -- is left alone too, since it means "no signal", not "released".
    local brake_out_pwm = brake_asleep and 0 or math.floor(cfg.BRK_MIN + cfg.BRK_MAX - final_brake_pwm)

    -- =========================================================================
    -- GAS: only ever an override to force idle/failsafe. When allowed, this
    -- script does not touch SERVO2 at all -- ArduPilot's native throttle
    -- mixer already produces 0 output for any stick at/below the idle zone.
    --
    -- Gas is allowed in ANY gear, including P/N -- client wants to be able to
    -- rev the engine in Park/Neutral to check it's actually getting RPM
    -- (gearbox is mechanically disengaged in P/N, so the vehicle can't move
    -- from this alone). Still forced to idle while a gear shift is actually
    -- in flight (gear_cmd_pending -- actuator moving/unconfirmed) or during
    -- the own-timer failsafe.
    -- =========================================================================
    local gas_allowed = (not failsafe_active) and (not gear_cmd_pending)
    if failsafe_active then
        SRV_Channels:set_output_pwm_chan_timeout(GAS_CHAN, math.floor(cfg.FS_THRPWM), OUT_TIMEOUT_MS)
    elseif not gas_allowed then
        SRV_Channels:set_output_pwm_chan_timeout(GAS_CHAN, GAS_IDLE_PWM, OUT_TIMEOUT_MS)
    end

    -- =========================================================================
    -- E. FRONT DIFF-LOCK -- RC edge-trigger, relay cutoff, auto-timeout
    -- =========================================================================
    local diflock_request = vdiflock >= DIFLOCK_ON_THRESH
    if diflock_request and not difflock_ch_prev then
        if can_gear_fresh and can_state.gear_valid and can_state.gear ~= "H" then
            relay:on(DIFLOCK_RELAY)
            difflock_active      = true
            difflock_deadline_ms = now + cfg.DL_WORKT * 1000
            log(SEV_INFO, "DifLock engaged")
        else
            warn_throttled("DifLock request blocked (gear H or unknown)")
        end
    end
    difflock_ch_prev = diflock_request

    if difflock_active and now > difflock_deadline_ms then
        relay:off(DIFLOCK_RELAY)
        difflock_active = false
        log(SEV_INFO, "DifLock auto-disabled (timeout)")
    end

    -- =========================================================================
    -- F. MODE-STATUS TELEMETRY -- consumed by Maul-Docker (mavlink_link.py)
    -- so the dashboard can show what this script is actually doing, not just
    -- raw CAN values. MAUL_DL is a continuous countdown (seconds, -1=idle);
    -- MAUL_ST is a bitmask of the discrete state-machine flags.
    -- =========================================================================
    if now - status_tlm_last_ms >= 1000 then
        status_tlm_last_ms = now
        local remaining = difflock_active and ((difflock_deadline_ms - now) / 1000.0) or -1
        gcs:send_named_float("MAUL_DL", remaining)

        local bits = 0
        if gear_wait_safe    then bits = bits | 0x01 end
        if gear_cmd_pending  then bits = bits | 0x02 end
        if gear_park_wait    then bits = bits | 0x04 end
        if handbrake_engaged then bits = bits | 0x08 end
        if hillhold_active   then bits = bits | 0x10 end
        if failsafe_active   then bits = bits | 0x20 end
        if difflock_active   then bits = bits | 0x40 end
        gcs:send_named_float("MAUL_ST", bits)

        -- Publish the CAN-bus 12V accessory-battery voltage (0x342 frame) as
        -- ArduPilot's own battery monitor 0 (BATT_MONITOR=29/Scripting, see
        -- defaults.parm) -- comes out as a normal BATTERY_STATUS message, so
        -- any GCS that already shows battery voltage needs no changes at all.
        -- Current/remaining-% are left unset (unknown) -- this CAN frame only
        -- carries voltage.
        local batt_state = BattMonitorScript_State()
        batt_state:healthy(now - can_state.ts_342 < cfg.CAN_TIMEOUT)
        batt_state:voltage(can_state.batt_voltage)
        battery:handle_scripting(0, batt_state)

        -- Publish engine rpm + cylinder head temperature as a native
        -- EFI_STATUS message (EFI_TYPE=7/Scripting, see defaults.parm). Both
        -- come from the same 0x102 CAN frame, so can_rpm_fresh covers both.
        -- cylinder_head_temperature is stored in Kelvin internally
        -- (AP_EFI_State.h), hence the +273.15 -- can_state.engine_temp_c is
        -- already Celsius (0x102 frame, byte3 - 60).
        --
        -- last_updated_ms is only bumped while can_rpm_fresh: AP_EFI derives
        -- is_healthy() purely from how recently it was called, so leaving it
        -- unset (stays 0 on this fresh EFI_State()) while the CAN frame is
        -- stale correctly marks the backend unhealthy instead of parroting
        -- the last good reading forever.
        local efi_backend = efi:get_backend(0)
        if efi_backend then
            local efi_state = EFI_State()
            efi_state:engine_speed_rpm(can_state.rpm)
            local cyl_status = Cylinder_Status()
            cyl_status:cylinder_head_temperature(can_state.engine_temp_c + 273.15)
            efi_state:cylinder_status(cyl_status)
            if can_rpm_fresh then
                efi_state:last_updated_ms(now)
            end
            efi_backend:handle_scripting(efi_state)
        end
    end

    -- =========================================================================
    -- D. STEERING -- speed attenuation ("progressive", opt-in) and, while
    -- diff-lock is active, a hard max-angle clamp
    -- =========================================================================
    if cfg.STR_ATT_EN ~= 0 or difflock_active then
        local steer_native = SRV_Channels:get_output_pwm(STEER_FUNC)
        if steer_native then
            local gain = 1.0
            if cfg.STR_ATT_EN ~= 0 then
                local speed_for_gain = can_speed_fresh and can_state.speed_kmh or cfg.STR_SPD_MAX
                gain = steering_gain(speed_for_gain)
            end
            local steer_pwm = steer_trim_pwm + (steer_native - steer_trim_pwm) * gain
            if difflock_active then
                steer_pwm = clamp(steer_pwm, steer_trim_pwm - cfg.STR_DLMXANG, steer_trim_pwm + cfg.STR_DLMXANG)
            end
            SRV_Channels:set_output_pwm_chan_timeout(STEER_CHAN, math.floor(steer_pwm), OUT_TIMEOUT_MS)
        end
    end

    -- =========================================================================
    -- OUTPUT
    -- =========================================================================
    SRV_Channels:set_output_pwm_chan_timeout(BRAKE_CHAN, brake_out_pwm, OUT_TIMEOUT_MS)

    if cfg.CAL_RUN == 0 then
        local gear_out_pwm
        if now - gear_cmd_start_ms > (cfg.GEAR_MAXT + cfg.GEAR_TOUT) then
            gear_out_pwm = 0  -- protect the actuator motor: stop driving it if a shift never settled
        else
            gear_out_pwm = commanded_gear_pwm
        end
        SRV_Channels:set_output_pwm_chan_timeout(GEAR_CHAN, math.floor(gear_out_pwm), OUT_TIMEOUT_MS)
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
    gear_cmd_start_ms   = millis()  -- so the protect-timeout counts from boot, not from an unset 0
    steer_trim_pwm      = math.floor(param:get('SERVO1_TRIM') or 1500)
    log(SEV_INFO, "MAUL_Control ready (gear/steer-angle params UNCALIBRATED until measured)")
    return update, 500
end

return init()
