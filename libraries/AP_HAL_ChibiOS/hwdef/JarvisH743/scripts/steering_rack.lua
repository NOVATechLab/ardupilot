-- Steering rack closed-loop control
-- Potentiometer: PB0 (ADC1_INP9)
-- RELAY1 (PE8) = LEFT,  RELAY2 (PE7) = RIGHT
-- Steering command: SRV_Channels output ch1 (GroundSteering, SERVO1_FUNCTION=26)

-- ── Parameters (visible in GCS under JRVS_ tab) ──────────────────────────
local PARAM_TABLE_KEY    = 72
local PARAM_TABLE_PREFIX = "JRVS_"

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 5),
    "RACK: failed to add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "RACK_V_MIN", 2.00), "RACK: param 1")
assert(param:add_param(PARAM_TABLE_KEY, 2, "RACK_V_CTR", 2.62), "RACK: param 2")
assert(param:add_param(PARAM_TABLE_KEY, 3, "RACK_V_MAX", 3.26), "RACK: param 3")
assert(param:add_param(PARAM_TABLE_KEY, 4, "RACK_DB_C",  0.07), "RACK: param 4")
assert(param:add_param(PARAM_TABLE_KEY, 5, "RACK_DB_M",  0.04), "RACK: param 5")

-- Full names: JRVS_RACK_V_MIN, JRVS_RACK_V_CTR, JRVS_RACK_V_MAX
--             JRVS_RACK_DB_C (deadband at center),  JRVS_RACK_DB_M (deadband while moving)
local p_v_min = Parameter(); p_v_min:init("JRVS_RACK_V_MIN")
local p_v_ctr = Parameter(); p_v_ctr:init("JRVS_RACK_V_CTR")
local p_v_max = Parameter(); p_v_max:init("JRVS_RACK_V_MAX")
local p_db_c  = Parameter(); p_db_c:init("JRVS_RACK_DB_C")
local p_db_m  = Parameter(); p_db_m:init("JRVS_RACK_DB_M")

-- ── Constants ─────────────────────────────────────────────────────────────
local RELAY_LEFT   = 0   -- RELAY1 = PE8 (relay instance is 0-based!)
local RELAY_RIGHT  = 1   -- RELAY2 = PE7
local POT_CHANNEL  = 9   -- ADC1_INP9 = PB0
local FUNC_STEERING = 26 -- SRV_Channel k_steering (GroundSteering), NOT a channel index
local PWM_MIN      = 1100
local PWM_CENTER   = 1500
local PWM_MAX      = 1900
local PWM_DEADBAND = 50  -- joystick center deadband (µs)
local HW_GUARD     = 0.05 -- stop relay when within 50mV of hard limit

local pot = analog:channel()
assert(pot:set_pin(POT_CHANNEL), "RACK: invalid analog pin")

local last_state = nil  -- "LEFT" / "RIGHT" / "CENTER" / "LIMIT_MAX" / "LIMIT_MIN"
local log_counter = 0   -- throttles the periodic GCS status message

-- Direction codes used in the dataflash STRK log
local DIR_STOP  = 0
local DIR_RIGHT = 1
local DIR_LEFT  = 2

-- ── Relay helpers ─────────────────────────────────────────────────────────
local function relay_stop()
    relay:off(RELAY_LEFT)
    relay:off(RELAY_RIGHT)
end

local function relay_left()
    relay:off(RELAY_RIGHT)
    relay:on(RELAY_LEFT)
end

local function relay_right()
    relay:off(RELAY_LEFT)
    relay:on(RELAY_RIGHT)
end

-- ── Map PWM → target pot voltage ─────────────────────────────────────────
local function get_target_v(pwm)
    local v_min = p_v_min:get()
    local v_ctr = p_v_ctr:get()
    local v_max = p_v_max:get()
    local cl = PWM_CENTER - PWM_DEADBAND
    local ch = PWM_CENTER + PWM_DEADBAND

    if pwm < cl then
        local t = (pwm - PWM_MIN) / (cl - PWM_MIN)
        return v_min + t * (v_ctr - v_min)
    elseif pwm > ch then
        local t = (pwm - ch) / (PWM_MAX - ch)
        return v_ctr + t * (v_max - v_ctr)
    else
        return v_ctr
    end
end

-- ── Main loop ─────────────────────────────────────────────────────────────
local function update()
    local pwm   = SRV_Channels:get_output_pwm(FUNC_STEERING)
    local pot_v = pot:voltage_average()
    local v_min = p_v_min:get()
    local v_max = p_v_max:get()

    -- no valid steering output yet (pre-arm / failsafe / unassigned) → hold
    if pwm == nil then
        relay_stop()
        if last_state ~= "NO_PWM" then
            gcs:send_text(4, "RACK: no steering output")
            last_state = "NO_PWM"
        end
        return update, 200
    end

    -- pot sanity check (disconnected or shorted)
    if pot_v < 0.1 or pot_v > 3.3 then
        relay_stop()
        gcs:send_text(4, string.format("RACK: pot fault %.2fV", pot_v))
        return update, 200
    end

    local target_v = get_target_v(pwm)
    local error_v  = target_v - pot_v
    local deadband = (math.abs(pwm - PWM_CENTER) < PWM_DEADBAND)
                     and p_db_c:get() or p_db_m:get()

    local dir = DIR_STOP

    if math.abs(error_v) <= deadband then
        relay_stop()
        dir = DIR_STOP
        if last_state ~= "CENTER" then
            gcs:send_text(6, string.format("RACK: centered pot=%.2fV", pot_v))
            last_state = "CENTER"
        end
    elseif error_v > 0 then
        if pot_v >= v_max - HW_GUARD then
            relay_stop()
            dir = DIR_STOP
            if last_state ~= "LIMIT_MAX" then
                gcs:send_text(4, string.format("RACK: max limit reached %.2fV", pot_v))
                last_state = "LIMIT_MAX"
            end
        else
            relay_right()
            dir = DIR_RIGHT
            if last_state ~= "RIGHT" then
                gcs:send_text(6, string.format(
                    "RACK: turn RIGHT pot=%.2fV tgt=%.2fV", pot_v, target_v))
                last_state = "RIGHT"
            end
        end
    else
        if pot_v <= v_min + HW_GUARD then
            relay_stop()
            dir = DIR_STOP
            if last_state ~= "LIMIT_MIN" then
                gcs:send_text(4, string.format("RACK: min limit reached %.2fV", pot_v))
                last_state = "LIMIT_MIN"
            end
        else
            relay_left()
            dir = DIR_LEFT
            if last_state ~= "LEFT" then
                gcs:send_text(6, string.format(
                    "RACK: turn LEFT pot=%.2fV tgt=%.2fV", pot_v, target_v))
                last_state = "LEFT"
            end
        end
    end

    -- ── Periodic GCS status (~1 Hz) so you can watch it live in Messages ──
    log_counter = log_counter + 1
    if log_counter >= 20 then
        log_counter = 0
        gcs:send_text(6, string.format(
            "RACK: pwm=%d pot=%.2fV tgt=%.2fV err=%+.2fV dir=%d",
            pwm, pot_v, target_v, error_v, dir))
    end

    return update, 50  -- 20 Hz
end

gcs:send_text(6, "steering_rack: ready | params JRVS_RACK_V_*/DB_*")
return update()
