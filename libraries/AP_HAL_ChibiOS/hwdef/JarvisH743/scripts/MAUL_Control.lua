-- =============================================================================
-- MAUL_Control.lua — Can-Am/BRP UGV brake/gear control + throttle gate (UGV_Maul)
-- =============================================================================
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
--      idle zone AND CAN-reported speed (0x231, from MAUL_CanDecode.lua) is
--      ~0.
--
-- CH_THROTTLE (RC_OVERRIDE): raw PWM read directly here for brake + interlock
--                             decisions ONLY. The actual gas PWM is computed
--                             by ArduPilot's native throttle mixer from the
--                             same physical channel (RCMAP_THROTTLE=1).
-- CH_GEAR     (RC_OVERRIDE): 5 zones -> P / N / R / L / H target
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
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 17), "MAUL: add_table failed")

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

        local can = MAUL_CAN
        if can and can.gear_valid and (now - can.ts_309 < cfg.CAN_TIMEOUT) then
            local w = cal_windows[can.gear]
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

    if not rc:has_valid_input() then
        -- fail safe: force gas idle (on top of ArduPilot's own RC failsafe),
        -- full brake, hold last commanded gear
        SRV_Channels:set_output_pwm_chan_timeout(GAS_CHAN,   GAS_IDLE_PWM, OUT_TIMEOUT_MS)
        SRV_Channels:set_output_pwm_chan_timeout(BRAKE_CHAN, cfg.BRK_MAX,  OUT_TIMEOUT_MS)
        SRV_Channels:set_output_pwm_chan_timeout(GEAR_CHAN,  commanded_gear_pwm, OUT_TIMEOUT_MS)
        return update, 20
    end

    local v1    = rc_pwm(CH_THROTTLE, cfg.THR_DZ_LO)  -- default: idle
    local vgear = rc_pwm(CH_GEAR, 1000)                -- default: P zone

    target_gear = target_gear_from_pwm(vgear)

    local can = MAUL_CAN
    local can_gear_fresh  = can and (now - can.ts_309 < cfg.CAN_TIMEOUT)
    local can_speed_fresh = can and (now - can.ts_231 < cfg.CAN_TIMEOUT)
    local confirmed_gear  = (can_gear_fresh and can.gear_valid) and can.gear or "UNKNOWN"
    local speed_ok         = can_speed_fresh and (can.speed_kmh < cfg.GEAR_SPD_MX)

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
            local speed_for_gain = can_speed_fresh and can.speed_kmh or cfg.STR_SPD_MAX
            local gain = steering_gain(speed_for_gain)
            local steer_pwm = steer_trim_pwm + (steer_native - steer_trim_pwm) * gain
            SRV_Channels:set_output_pwm_chan_timeout(STEER_CHAN, math.floor(steer_pwm), OUT_TIMEOUT_MS)
        end
    end

    SRV_Channels:set_output_pwm_chan_timeout(BRAKE_CHAN, math.floor(brake_pwm), OUT_TIMEOUT_MS)
    if cfg.CAL_RUN == 0 then
        SRV_Channels:set_output_pwm_chan_timeout(GEAR_CHAN, math.floor(commanded_gear_pwm), OUT_TIMEOUT_MS)
    end

    return update, 20
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
