-- =============================================================================
-- JRVS_MamontUGV.lua  v2  —  Hydraulic rover controller for JarvisH743
-- =============================================================================
-- ArduPilot controls throttle (SERVO1, k_throttle, MOT_PWM_TYPE=4, 1kHz).
-- Lua manages only: relays, hydraulic brakes, gear (THR_MAX).
--
-- CH1 Throttle  : >1500 gas | 1400-1500 idle | <1400 full brake
-- CH2 Steering  : <1380 left | >1620 right | center free
-- CH3 Battery   : >1700 ON (hold 1s)
-- CH4 Gear      : <1200 G1 | <1700 G2 | ≥1700 G3
-- CH5 Selector  : >1700 reverse (hold 1s)
-- CH6 Body      : >1700 up | <1300 down | center stop
-- =============================================================================

-- ---------------------------------------------------------------------------
-- RELAY MAPPING  (0-based indices)
-- ---------------------------------------------------------------------------
--  0  RELAY1 PE8  — ReleBtr   battery relay
--  1  RELAY2 PE7  — ReleSlc   selector (forward/reverse)
--  2  RELAY3 PB2  — RelePump  hydraulic pump
--  3  RELAY4 PB1  — ReleLPV   left pressure valve
--  4  RELAY5 PB0  — ReleRPV   right pressure valve
--  5  RELAY6 PD12 — ReleLRV   left release valve / body up
--  6  RELAY7 PD13 — ReleRRV   right release valve / body down
--  7  RELAY8 PD15 — ReleSsvl  hydraulics switch: OFF=brakes ON=body
local IDX_BTR  = 0
local IDX_SLC  = 1
local IDX_PUMP = 2
local IDX_LPV  = 3
local IDX_RPV  = 4
local IDX_LRV  = 5
local IDX_RRV  = 6
local IDX_SSVL = 7

local function relay_on(idx)  relay:on(idx)  end
local function relay_off(idx) relay:off(idx) end

-- ---------------------------------------------------------------------------
-- LOGGING
-- ---------------------------------------------------------------------------
local SEV_INFO = 6
local SEV_WARN = 4
local function log(sev, msg) gcs:send_text(sev, "JRVS: " .. msg) end

-- ---------------------------------------------------------------------------
-- CUSTOM PARAMETERS
-- ---------------------------------------------------------------------------
local PARAM_TABLE_KEY = 72
assert(param:add_table(PARAM_TABLE_KEY, "JRVS_", 10), "JRVS: param table failed")

assert(param:add_param(PARAM_TABLE_KEY, 1, 'G1_SVO',   1200))
assert(param:add_param(PARAM_TABLE_KEY, 2, 'G2_SVO',   1350))
assert(param:add_param(PARAM_TABLE_KEY, 3, 'G3_SVO',   1600))
assert(param:add_param(PARAM_TABLE_KEY, 4, 'G1_THR',   20))
assert(param:add_param(PARAM_TABLE_KEY, 5, 'G2_THR',   35))
assert(param:add_param(PARAM_TABLE_KEY, 6, 'G3_THR',   60))
assert(param:add_param(PARAM_TABLE_KEY, 7, 'BRK_HOLD', 350))
assert(param:add_param(PARAM_TABLE_KEY, 8, 'BRK_REL',  400))
assert(param:add_param(PARAM_TABLE_KEY, 9, 'BRK_PLS',  80))

local p_g1_svo   = Parameter('JRVS_G1_SVO')
local p_g2_svo   = Parameter('JRVS_G2_SVO')
local p_g3_svo   = Parameter('JRVS_G3_SVO')
local p_g1_thr   = Parameter('JRVS_G1_THR')
local p_g2_thr   = Parameter('JRVS_G2_THR')
local p_g3_thr   = Parameter('JRVS_G3_THR')
local p_brk_hold = Parameter('JRVS_BRK_HOLD')
local p_brk_rel  = Parameter('JRVS_BRK_REL')
local p_brk_pls  = Parameter('JRVS_BRK_PLS')

-- ---------------------------------------------------------------------------
-- RC CHANNELS
-- ---------------------------------------------------------------------------
local CH_VERT = 1
local CH_HORT = 2
local CH_BTR  = 3
local CH_GEAR = 4
local CH_SLC  = 5
local CH_BODY = 6

-- ---------------------------------------------------------------------------
-- GEARS  →  SERVO1_MAX
-- ---------------------------------------------------------------------------
local GEAR_SVO = { p_g1_svo, p_g2_svo, p_g3_svo }
local GEAR_THR = { p_g1_thr, p_g2_thr, p_g3_thr }
local nGear    = 1

local function update_gear(pwm)
    local new_gear
    if pwm < 1200 then
        new_gear = 1
    elseif pwm < 1700 then
        new_gear = 2
    else
        new_gear = 3
    end
    if new_gear ~= nGear then
        nGear = new_gear
        local svo = GEAR_SVO[nGear]:get()
        local thr = GEAR_THR[nGear]:get()
        param:set('SERVO1_MAX',  svo)
        param:set('MOT_THR_MAX', thr)
        log(SEV_INFO, string.format("Gear %d  SERVO1_MAX=%d  MOT_THR_MAX=%d", nGear, svo, thr))
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
        if not btr_armed then
            btr_timer = now
            btr_armed = true
        end
        if now - btr_timer >= RELAY_HOLD_MS and not btr_state then
            relay_on(IDX_BTR)
            btr_state = true
            log(SEV_INFO, "Battery relay ON")
        end
    else
        btr_armed = false
        if btr_state then
            relay_off(IDX_BTR)
            btr_state = false
            log(SEV_INFO, "Battery relay OFF")
        end
    end
end

-- ---------------------------------------------------------------------------
-- SELECTOR RELAY  (hold 1s before switching)
-- ---------------------------------------------------------------------------
local slc_timer = 0
local slc_armed = false
local slc_state = false

local function update_selector(v, now)
    if v > 1700 then
        if not slc_armed then
            slc_timer = now
            slc_armed = true
        end
        if now - slc_timer >= RELAY_HOLD_MS and not slc_state then
            relay_on(IDX_SLC)
            slc_state = true
            log(SEV_INFO, "Selector: reverse")
        end
    else
        slc_armed = false
        if slc_state then
            relay_off(IDX_SLC)
            slc_state = false
            log(SEV_INFO, "Selector: forward")
        end
    end
end

-- ---------------------------------------------------------------------------
-- HYDRAULIC BRAKES
-- ---------------------------------------------------------------------------
-- brake timing comes from JRVS_BRK_HOLD / BRK_REL / BRK_PLS parameters

local brake_left_on   = false
local brake_right_on  = false
local brake_start_ms  = 0
local brake_pulse_ms  = 0
local brake_releasing = false

local function _set_release_valves(left, right, open)
    if left  then if open then relay_on(IDX_LRV) else relay_off(IDX_LRV) end end
    if right then if open then relay_on(IDX_RRV) else relay_off(IDX_RRV) end end
end

local function do_brake(left, right, now)
    local changed = (left ~= brake_left_on) or (right ~= brake_right_on)
    if changed then
        if brake_left_on  and not left  then relay_on(IDX_LRV); relay_off(IDX_LPV) end
        if brake_right_on and not right then relay_on(IDX_RRV); relay_off(IDX_RPV) end
        brake_start_ms  = now
        brake_pulse_ms  = now
        brake_releasing = false
        brake_left_on   = left
        brake_right_on  = right
    end

    relay_on(IDX_PUMP)
    if left  then relay_on(IDX_LPV) end
    if right then relay_on(IDX_RPV) end

    if now - brake_start_ms < p_brk_hold:get() then
        _set_release_valves(left, right, false)
        return
    end

    local interval = brake_releasing and p_brk_rel:get() or p_brk_pls:get()
    if now - brake_pulse_ms >= interval then
        brake_pulse_ms  = now
        brake_releasing = not brake_releasing
        _set_release_valves(left, right, brake_releasing)
    end
end

local function un_brake()
    -- always open release valves and kill pressure — prevents brake lockup from body mode
    relay_on(IDX_LRV);  relay_on(IDX_RRV)
    relay_off(IDX_PUMP)
    relay_off(IDX_LPV); relay_off(IDX_RPV)
    if brake_left_on or brake_right_on then
        brake_left_on   = false
        brake_right_on  = false
        brake_releasing = false
        brake_pulse_ms  = 0
        log(SEV_INFO, "Brake released")
    end
end

-- Simple relay toggle for steering turns (no pulse timing)
local function do_turn_brake(left, right)
    relay_on(IDX_PUMP)
    if left  then relay_on(IDX_LPV);  relay_off(IDX_LRV)
    else          relay_off(IDX_LPV); relay_on(IDX_LRV) end
    if right then relay_on(IDX_RPV);  relay_off(IDX_RRV)
    else          relay_off(IDX_RPV); relay_on(IDX_RRV) end
    brake_left_on  = left
    brake_right_on = right
end

-- ---------------------------------------------------------------------------
-- BODY (dump truck)
-- ---------------------------------------------------------------------------
local body_state = "STOP"

local function update_body(v, motor_idle)
    local new_state
    if not motor_idle then
        -- motor running: body disabled, but don't touch brake relays (PUMP/LRV/RRV)
        new_state = "LOCKED"
        relay_off(IDX_SSVL)
    elseif v > 1700 then
        new_state = "UP"
        relay_on(IDX_SSVL)
        relay_on(IDX_PUMP)
        relay_on(IDX_LRV)
        relay_off(IDX_RRV)
    elseif v < 1300 then
        new_state = "DOWN"
        relay_on(IDX_SSVL)
        relay_on(IDX_PUMP)
        relay_off(IDX_LRV)
        relay_on(IDX_RRV)
    else
        -- body idle: switch off body hydraulics but don't touch LRV/RRV (brake system owns them)
        new_state = "STOP"
        relay_off(IDX_SSVL)
        relay_off(IDX_PUMP)
    end
    if new_state ~= body_state then
        log(SEV_INFO, "Body: " .. new_state)
        body_state = new_state
    end
end

-- ---------------------------------------------------------------------------
-- MAIN LOOP
-- ---------------------------------------------------------------------------
local prev_mode = "INIT"

local function update()
    local now  = millis()
    local v1   = rc:get_pwm(CH_VERT) or 1500
    local v2   = rc:get_pwm(CH_HORT) or 1500
    local v_bt = rc:get_pwm(CH_BTR)  or 0
    local v_gr = rc:get_pwm(CH_GEAR) or 1000
    local v_sl = rc:get_pwm(CH_SLC)  or 0
    local v_bd = rc:get_pwm(CH_BODY) or 1500

    update_gear(v_gr)
    update_battery(v_bt, now)
    update_selector(v_sl, now)

    -- -----------------------------------------------------------------------
    -- THROTTLE / BRAKE / STEER
    -- -----------------------------------------------------------------------
    local mode

    if v1 < 1400 then
        -- emergency brake: pulse logic to manage hydraulic pressure
        do_brake(true, true, now)
        mode = "BRAKE"
    else
        -- steering: CH2 independently toggles one-side brake
        if v2 < 1380 then
            do_turn_brake(true, false)
            mode = "TURN_L"
        elseif v2 > 1620 then
            do_turn_brake(false, true)
            mode = "TURN_R"
        else
            un_brake()
            mode = v1 > 1510 and "GAS" or "IDLE"
        end
    end

    -- Body: дозволено тільки на повному холостому ходу
    local motor_idle = (v1 >= 1400 and v1 <= 1510)
    update_body(v_bd, motor_idle)

    if mode ~= prev_mode then
        log(SEV_INFO, string.format("Mode: %s  v1=%d v2=%d G=%d", mode, v1, v2, nGear))
        prev_mode = mode
    end

    return update, 20
end

-- ---------------------------------------------------------------------------
-- INIT
-- ---------------------------------------------------------------------------
local function init()
    for i = 0, 7 do relay_off(i) end
    local svo = GEAR_SVO[nGear]:get()
    local thr = GEAR_THR[nGear]:get()
    param:set('SERVO1_MAX',  svo)
    param:set('MOT_THR_MAX', thr)
    log(SEV_INFO, string.format("JRVS_MamontUGV v2  gear=%d  SERVO1_MAX=%d  MOT_THR_MAX=%d",
        nGear, svo, thr))
    return update, 500
end

return init()
