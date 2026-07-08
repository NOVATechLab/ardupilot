-- =============================================================================
-- JRVS_MamontUGV.lua  V2  —  Amphibious hydraulic UGV controller for JarvisH743
-- =============================================================================
-- ArduPilot controls throttle (SERVO1, k_throttle, MOT_PWM_TYPE=4, 1kHz).
-- Lua manages: 16 relays, hydraulic brakes, gears (SERVO1_MAX/MOT_THR_MAX),
-- propellers (swim), body/blade/light/reserve aux, drive modes.
--
-- CH1  Throttle  : >1510 gas | 1400-1510 idle | <1400 full brake
-- CH2  Steering  : <1380 left | >1620 right | center free
-- CH3  (unused — left free to avoid QGC conflicts)
-- CH4  (unused — left free to avoid QGC conflicts)
-- CH5  Selector  : >1700 reverse (hold 1s)
-- CH6  Mode      : <1200 LAND | <1700 COMBI | >=1700 SWIM
-- CH7  Body      : >1700 up (RELAY10) | <1300 down (RELAY11) | center off
-- CH8  Blade     : >1700 up (RELAY12) | <1300 down (RELAY13) | center off
-- CH9  Light     : >1700 on  (RELAY14) | else off
-- CH10 Reserve   : >1700 up (RELAY15) | <1300 down (RELAY16) | center off
-- CH11 Battery   : >1700 ON  (hold 1s)
-- CH12 Gear      : <1200 G1 | <1700 G2 | >=1700 G3
-- =============================================================================

-- ---------------------------------------------------------------------------
-- RELAY MAPPING  (0-based indices, see hwdef RELAYn_PIN_DEFAULT)
-- ---------------------------------------------------------------------------
--  0 RELAY1  PE8  ReleBtr   battery
--  1 RELAY2  PE7  ReleSlc   selector forward/reverse
--  2 RELAY3  PB2  RelePump  hydraulic pump (shared: brakes + body/blade)
--  3 RELAY4  PB1  ReleLPV   left  pressure valve
--  4 RELAY5  PB0  ReleRPV   right pressure valve
--  5 RELAY6  PD12 left  propeller FORWARD
--  6 RELAY7  PD13 left  propeller REVERSE
--  7 RELAY8  PD15 right propeller FORWARD
--  8 RELAY9  PA1  right propeller REVERSE
--  9 RELAY10 PA2  body  UP
-- 10 RELAY11 PA3  body  DOWN
-- 11 RELAY12 PE10 blade UP
-- 12 RELAY13 PE11 blade DOWN
-- 13 RELAY14 PE12 light
-- 14 RELAY15 PE13 reserve UP / +
-- 15 RELAY16 PD14 reserve DOWN / -
local IDX_BTR     = 0
local IDX_SLC     = 1
local IDX_PUMP    = 2
local IDX_LPV     = 3
local IDX_RPV     = 4
local IDX_LPROP_F = 5
local IDX_LPROP_R = 6
local IDX_RPROP_F = 7
local IDX_RPROP_R = 8
local IDX_BODY_UP = 9
local IDX_BODY_DN = 10
local IDX_BLADE_UP = 11
local IDX_BLADE_DN = 12
local IDX_LIGHT   = 13
local IDX_RSV_UP  = 14
local IDX_RSV_DN  = 15
local RELAY_LAST  = 15

local function relay_on(idx)  relay:on(idx)  end
local function relay_off(idx) relay:off(idx) end

-- ---------------------------------------------------------------------------
-- LOGGING
-- ---------------------------------------------------------------------------
local SEV_INFO = 6
local function log(sev, msg) gcs:send_text(sev, "JRVS: " .. msg) end

-- ---------------------------------------------------------------------------
-- RC CHANNELS
-- ---------------------------------------------------------------------------
local CH_VERT = 1   -- throttle / brake
local CH_HORT = 2   -- steering
-- CH3, CH4 left free (avoid QGC conflicts); battery/gear moved to CH11/CH12
local CH_SLC  = 5   -- selector fwd/rev
local CH_MODE = 6   -- drive mode
local CH_BODY = 7
local CH_BLADE = 8
local CH_LIGHT = 9
local CH_RSV  = 10
local CH_BTR  = 11  -- battery
local CH_GEAR = 12  -- gear

-- ---------------------------------------------------------------------------
-- CUSTOM PARAMETERS  (table key 72, prefix JRVS_)
-- ---------------------------------------------------------------------------
local PARAM_TABLE_KEY    = 72
local PARAM_TABLE_PREFIX = "JRVS_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 9), "JRVS: add_table failed")

local function add_param(idx, name, default)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default), "JRVS: add_param " .. name)
    local p = Parameter()
    assert(p:init(PARAM_TABLE_PREFIX .. name), "JRVS: init " .. name)
    return p
end

local P_G1_SVO   = add_param(1, 'G1_SVO',   1200)  -- SERVO1_MAX for G1 (us)
local P_G2_SVO   = add_param(2, 'G2_SVO',   1350)  -- SERVO1_MAX for G2 (us)
local P_G3_SVO   = add_param(3, 'G3_SVO',   1600)  -- SERVO1_MAX for G3 (us)
local P_G1_THR   = add_param(4, 'G1_THR',   20)    -- MOT_THR_MAX for G1 (%)
local P_G2_THR   = add_param(5, 'G2_THR',   35)    -- MOT_THR_MAX for G2 (%)
local P_G3_THR   = add_param(6, 'G3_THR',   60)    -- MOT_THR_MAX for G3 (%)
local P_BRK_HOLD = add_param(7, 'BRK_HOLD', 350)   -- pressure hold before pulsing (ms)
local P_BRK_REL  = add_param(8, 'BRK_REL',  400)   -- pulse release phase (ms)
local P_BRK_PLS  = add_param(9, 'BRK_PLS',  80)    -- pulse hold phase (ms)

-- Per-cycle cache of parameter values (read once per update, not per use)
local cfg = {}
local function refresh_cfg()
    cfg.G1_SVO   = P_G1_SVO:get()
    cfg.G2_SVO   = P_G2_SVO:get()
    cfg.G3_SVO   = P_G3_SVO:get()
    cfg.G1_THR   = P_G1_THR:get()
    cfg.G2_THR   = P_G2_THR:get()
    cfg.G3_THR   = P_G3_THR:get()
    cfg.BRK_HOLD = P_BRK_HOLD:get()
    cfg.BRK_REL  = P_BRK_REL:get()
    cfg.BRK_PLS  = P_BRK_PLS:get()
end

-- ---------------------------------------------------------------------------
-- GEARS  ->  SERVO1_MAX / MOT_THR_MAX
-- ---------------------------------------------------------------------------
local nGear = 1

local function apply_gear()
    local svo = ({ cfg.G1_SVO, cfg.G2_SVO, cfg.G3_SVO })[nGear]
    local thr = ({ cfg.G1_THR, cfg.G2_THR, cfg.G3_THR })[nGear]
    param:set('SERVO1_MAX',  svo)
    param:set('MOT_THR_MAX', thr)
    log(SEV_INFO, string.format("Gear %d  SERVO1_MAX=%d  MOT_THR_MAX=%d", nGear, svo, thr))
end

local function update_gear(pwm)
    local g
    if pwm < 1200 then g = 1 elseif pwm < 1700 then g = 2 else g = 3 end
    if g ~= nGear then
        nGear = g
        apply_gear()
    end
end

-- ---------------------------------------------------------------------------
-- BATTERY RELAY  (hold 1s before switching)
-- ---------------------------------------------------------------------------
local RELAY_HOLD_MS = 1000
local btr_timer = 0
local btr_armed = false
local btr_state = false

local function update_battery(v, now)
    if v > 1700 then
        if not btr_armed then btr_timer = now; btr_armed = true end
        if now - btr_timer >= RELAY_HOLD_MS and not btr_state then
            relay_on(IDX_BTR); btr_state = true
            log(SEV_INFO, "Battery relay ON")
        end
    else
        btr_armed = false
        if btr_state then
            relay_off(IDX_BTR); btr_state = false
            log(SEV_INFO, "Battery relay OFF")
        end
    end
end

-- ---------------------------------------------------------------------------
-- SELECTOR RELAY  (hold 1s before switching)
-- ---------------------------------------------------------------------------
local slc_timer = 0
local slc_armed = false
local slc_state = false   -- true = reverse

local function update_selector(v, now)
    if v > 1700 then
        if not slc_armed then slc_timer = now; slc_armed = true end
        if now - slc_timer >= RELAY_HOLD_MS and not slc_state then
            relay_on(IDX_SLC); slc_state = true
            log(SEV_INFO, "Selector: reverse")
        end
    else
        slc_armed = false
        if slc_state then
            relay_off(IDX_SLC); slc_state = false
            log(SEV_INFO, "Selector: forward")
        end
    end
end

-- ---------------------------------------------------------------------------
-- HYDRAULIC BRAKES (V2: pulse the pressure valves LPV/RPV directly —
--                   there are no separate release valves in V2)
-- ---------------------------------------------------------------------------
local brake_left_on   = false
local brake_right_on  = false
local brake_start_ms  = 0
local brake_pulse_ms  = 0
local brake_releasing = false   -- true while pressure dropped in a pulse

local function _set_pressure(left, right, on)
    if left  then if on then relay_on(IDX_LPV) else relay_off(IDX_LPV) end end
    if right then if on then relay_on(IDX_RPV) else relay_off(IDX_RPV) end end
end

-- Emergency / full brake with anti-lock pulsation on both sides.
local function do_brake(left, right, now)
    local changed = (left ~= brake_left_on) or (right ~= brake_right_on)
    if changed then
        brake_start_ms  = now
        brake_pulse_ms  = now
        brake_releasing = false
        brake_left_on   = left
        brake_right_on  = right
    end

    -- initial full-pressure hold
    if now - brake_start_ms < cfg.BRK_HOLD then
        _set_pressure(left, right, true)
        return
    end

    -- anti-lock pulsation: alternate release / hold phases
    local interval = brake_releasing and cfg.BRK_REL or cfg.BRK_PLS
    if now - brake_pulse_ms >= interval then
        brake_pulse_ms  = now
        brake_releasing = not brake_releasing
    end
    _set_pressure(left, right, not brake_releasing)
end

local function un_brake()
    if brake_left_on or brake_right_on then
        relay_off(IDX_LPV); relay_off(IDX_RPV)
        brake_left_on   = false
        brake_right_on  = false
        brake_releasing = false
        brake_pulse_ms  = 0
        log(SEV_INFO, "Brake released")
    end
end

-- Simple one-side pressure for steering turns (no pulsation).
local function do_turn_brake(left, right)
    if left  then relay_on(IDX_LPV)  else relay_off(IDX_LPV) end
    if right then relay_on(IDX_RPV)  else relay_off(IDX_RPV) end
    brake_left_on  = left
    brake_right_on = right
end

-- ---------------------------------------------------------------------------
-- PROPELLERS (swim)
-- ---------------------------------------------------------------------------
local function set_prop(fwd_idx, rev_idx, dir)
    if dir > 0 then
        relay_on(fwd_idx);  relay_off(rev_idx)
    elseif dir < 0 then
        relay_off(fwd_idx); relay_on(rev_idx)
    else
        relay_off(fwd_idx); relay_off(rev_idx)
    end
end

local function all_props_off()
    relay_off(IDX_LPROP_F); relay_off(IDX_LPROP_R)
    relay_off(IDX_RPROP_F); relay_off(IDX_RPROP_R)
end

-- Drive propellers from CH1/CH2. Returns true if any propeller is active.
local function swim_drive(v1, v2)
    local l, r = 0, 0
    if v2 < 1380 then          -- turn left:  L back, R fwd
        l, r = -1, 1
    elseif v2 > 1620 then      -- turn right: R back, L fwd
        l, r = 1, -1
    elseif v1 > 1510 then      -- gas: both forward
        l, r = 1, 1
    elseif v1 < 1400 then      -- water brake: both reverse
        l, r = -1, -1
    end
    if slc_state then l, r = -l, -r end   -- reverse selector inverts everything
    set_prop(IDX_LPROP_F, IDX_LPROP_R, l)
    set_prop(IDX_RPROP_F, IDX_RPROP_R, r)
    return (l ~= 0 or r ~= 0)
end

-- ---------------------------------------------------------------------------
-- LAND DRIVE (throttle via ArduPilot, hydraulic brakes/turns via Lua)
-- ---------------------------------------------------------------------------
local land_mode = "INIT"
local prev_land_mode = "INIT"

-- Apply land braking/steering from CH1/CH2. Returns true if pump pressure needed.
local function land_drive(v1, v2, now)
    local hyd = false
    if v1 < 1400 then
        do_brake(true, true, now)
        land_mode = "BRAKE"
        hyd = true
    elseif v2 < 1380 then
        do_turn_brake(true, false)
        land_mode = "TURN_L"
        hyd = true
    elseif v2 > 1620 then
        do_turn_brake(false, true)
        land_mode = "TURN_R"
        hyd = true
    else
        un_brake()
        land_mode = (v1 > 1510) and "GAS" or "IDLE"
    end
    if land_mode ~= prev_land_mode then
        log(SEV_INFO, string.format("Land: %s  v1=%d v2=%d G=%d", land_mode, v1, v2, nGear))
        prev_land_mode = land_mode
    end
    return hyd
end

-- ---------------------------------------------------------------------------
-- THROTTLE NEUTRALISATION (swim: lock the ground drive)
-- ---------------------------------------------------------------------------
local SERVO1_CHAN = 0    -- SERVO1 -> output channel index 0
local servo1_trim = 1000
local function neutralize_throttle()
    SRV_Channels:set_output_pwm_chan_timeout(SERVO1_CHAN, servo1_trim, 200)
end

-- ---------------------------------------------------------------------------
-- AUX (CH7..CH10) — work in any mode / any throttle
-- ---------------------------------------------------------------------------
-- up/down pair with mutual exclusion. Returns true if an actuator is driven.
local function aux_pair(v, up_idx, dn_idx)
    if v > 1700 then
        relay_on(up_idx);  relay_off(dn_idx); return true
    elseif v < 1300 then
        relay_off(up_idx); relay_on(dn_idx);  return true
    else
        relay_off(up_idx); relay_off(dn_idx); return false
    end
end

local function aux_light(v)
    if v > 1700 then relay_on(IDX_LIGHT) else relay_off(IDX_LIGHT) end
end

-- ---------------------------------------------------------------------------
-- MODES
-- ---------------------------------------------------------------------------
local MODE_LAND, MODE_COMBI, MODE_SWIM = "LAND", "COMBI", "SWIM"
local function read_mode(v)
    if v < 1200 then return MODE_LAND
    elseif v < 1700 then return MODE_COMBI
    else return MODE_SWIM end
end

-- ---------------------------------------------------------------------------
-- MAIN LOOP
-- ---------------------------------------------------------------------------
local prev_mode = "INIT"
local rc_ok = true   -- last known RC-input validity

-- Read an RC channel with a SAFE default. rc:get_pwm() returns 0 (not nil) for a
-- channel with no signal / not assigned on the transmitter, and "0 or default"
-- keeps that 0 in Lua. Anything below the valid RC range (<900) is treated as
-- "no channel" and replaced by `default` so unassigned aux channels don't latch.
local function rc_pwm(chan, default)
    local v = rc:get_pwm(chan)
    if not v or v < 900 then return default end
    return v
end

local function update()
    local now = millis()

    -- RC FAILSAFE. With no valid RC input rc:get_pwm() returns 0 (not nil), and
    -- in Lua "0 or 1500" == 0, so throttle reads 0 → land_drive() treats it as
    -- FULL BRAKE and pulses the pressure valves forever (audible clicking, welds
    -- contacts). Guard here: while input is invalid, hold every motion output in
    -- a safe state and skip the drive logic. Battery/selector relays are left as
    -- they are so an RC glitch does not cut power.
    if not rc:has_valid_input() then
        neutralize_throttle()
        if rc_ok then
            un_brake(); all_props_off()
            relay_off(IDX_PUMP)
            relay_off(IDX_BODY_UP);  relay_off(IDX_BODY_DN)
            relay_off(IDX_BLADE_UP); relay_off(IDX_BLADE_DN)
            relay_off(IDX_RSV_UP);   relay_off(IDX_RSV_DN)
            prev_mode = "INIT"; prev_land_mode = "INIT"
            log(SEV_INFO, "RC lost - outputs held safe")
            rc_ok = false
        end
        return update, 100
    end
    if not rc_ok then
        log(SEV_INFO, "RC input restored")
        rc_ok = true
    end

    refresh_cfg()

    local v1  = rc_pwm(CH_VERT,  1500)  -- center = idle
    local v2  = rc_pwm(CH_HORT,  1500)  -- center = straight
    local vbt = rc_pwm(CH_BTR,   1000)  -- low = battery OFF
    local vgr = rc_pwm(CH_GEAR,  1000)  -- low = G1
    local vsl = rc_pwm(CH_SLC,   1000)  -- low = forward
    local vmd = rc_pwm(CH_MODE,  1000)  -- low = LAND
    local v7  = rc_pwm(CH_BODY,  1500)  -- center = body off
    local v8  = rc_pwm(CH_BLADE, 1500)  -- center = blade off
    local v9  = rc_pwm(CH_LIGHT, 1000)  -- low = light OFF
    local v10 = rc_pwm(CH_RSV,   1500)  -- center = reserve off

    update_gear(vgr)
    update_battery(vbt, now)
    update_selector(vsl, now)

    local mode = read_mode(vmd)
    if mode ~= prev_mode then
        -- safe reset of previous mode's outputs before switching
        all_props_off()
        un_brake()
        log(SEV_INFO, "Mode: " .. mode)
        prev_mode = mode
    end

    -- drive logic per mode
    local hyd = false
    if mode == MODE_LAND then
        hyd = land_drive(v1, v2, now)
    elseif mode == MODE_SWIM then
        swim_drive(v1, v2)
        neutralize_throttle()
    else -- MODE_COMBI: land + swim simultaneously
        hyd = land_drive(v1, v2, now)
        swim_drive(v1, v2)
    end

    -- AUX (any mode)
    local aux_hyd = false
    if aux_pair(v7,  IDX_BODY_UP, IDX_BODY_DN)   then aux_hyd = true end
    if aux_pair(v8,  IDX_BLADE_UP, IDX_BLADE_DN) then aux_hyd = true end
    aux_light(v9)
    if aux_pair(v10, IDX_RSV_UP, IDX_RSV_DN)     then aux_hyd = true end

    -- shared hydraulic pump: ON for land braking/turning or body/blade/reserve
    if hyd or aux_hyd then relay_on(IDX_PUMP) else relay_off(IDX_PUMP) end

    return update, 20
end

-- ---------------------------------------------------------------------------
-- INIT
-- ---------------------------------------------------------------------------
local function init()
    for i = 0, RELAY_LAST do relay_off(i) end
    refresh_cfg()
    servo1_trim = math.floor(param:get('SERVO1_TRIM') or 1000)
    nGear = 1
    apply_gear()
    log(SEV_INFO, string.format("JRVS_MamontUGV V2 ready  gear=%d  trim=%d", nGear, servo1_trim))
    return update, 500
end

return init()
