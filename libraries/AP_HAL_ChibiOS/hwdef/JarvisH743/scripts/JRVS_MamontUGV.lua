-- =============================================================================
-- rover_control.lua  —  Hydraulic rover controller for JarvisH743
-- =============================================================================
-- Повна заміна Arduino-скетчу.  Керує гідравлічним ровером з:
--   • аналоговим газом (ШІМ на регулятор мотора)
--   • гідравлічними гальмами (насос + клапани)
--   • диференційним гальмуванням при розворотах
--   • реле батареї і селектора ходу (з затримкою 1 с)
--   • 3-ступінчастою трансмісією
-- =============================================================================

-- ---------------------------------------------------------------------------
-- RELAY MAPPING  (індекси 0-based для relay:on / relay:off)
-- ---------------------------------------------------------------------------
--  Індекс | RELAY# | GPIO | Пін  | Функція
--    0    | RELAY1 |  75  | PE8  | ReleBtr  — реле батареї
--    1    | RELAY2 |  74  | PE7  | ReleSlc  — реле селектора ходу вперед/назад
--    2    | RELAY3 |  73  | PB2  | RelePump — реле гідравлічного насоса
--    3    | RELAY4 |  72  | PB1  | ReleLPV  — лівий клапан тиску
--    4    | RELAY5 |  71  | PB0  | ReleRPV  — правий клапан тиску
--    5    | RELAY6 |  59  | PD12 | ReleLRV  — лівий клапан скидання / підняти кузов
--    6    | RELAY7 |  60  | PD13 | ReleRRV  — правий клапан скидання / опустити кузов
--    7    | RELAY8 |  62  | PD15 | ReleSsvl — реле перемикача гідравліки на кузов
-- ---------------------------------------------------------------------------
-- Реле АКТИВНО-ВИСОКІ (HIGH = реле ввімкнуто, підтяжка до 1):
--   relay:on(idx)  → пін HIGH → реле ввімкнено
--   relay:off(idx) → пін LOW  → реле вимкнено
-- ---------------------------------------------------------------------------
local IDX_BTR  = 0
local IDX_SLC  = 1
local IDX_PUMP = 2
local IDX_LPV  = 3
local IDX_RPV  = 4
local IDX_LRV  = 5  -- подвійна роль: клапан скидання гальм / підняти кузов
local IDX_RRV  = 6  -- подвійна роль: клапан скидання гальм / опустити кузов
local IDX_SSVL = 7  -- перемикач гідравліки: OFF=гальма, ON=кузов

-- Активно-високі реле: HIGH = ввімкнено
local function relay_on(idx)  relay:on(idx)  end  -- пін HIGH → реле ввімкнено
local function relay_off(idx) relay:off(idx) end  -- пін LOW  → реле вимкнено

-- ---------------------------------------------------------------------------
-- ЛОГУВАННЯ (через GCS) — тільки на переходах стану, не щоциклу
-- ---------------------------------------------------------------------------
-- MAV severity:  0=EMERGENCY  4=WARNING  6=INFO  7=DEBUG
local SEV_INFO = 6
local SEV_WARN = 4
local function log(sev, msg) gcs:send_text(sev, "JRVS: " .. msg) end

-- ---------------------------------------------------------------------------
-- RC CHANNELS  (1-based)
-- ---------------------------------------------------------------------------
--  CH1 — chVert  — газ (1500..2000) / гальмо (1000..1400)
--  CH2 — chHort  — поворот вліво (1000..1380) / вправо (1600..2000)
--  CH3 — ch3     — увімкнення батареї (>1100 тримати 1 с)
--  CH4 — ch4     — перемикання передач (перетин 1200 мкс)
--  CH5 — ch5     — селектор ходу вперед/назад (>1100 тримати 1 с)
--  CH6 — ch6     — кузов: >1600=підняти, <1400=опустити, центр=стоп
local CH_VERT = 1
local CH_HORT = 2
local CH_BTR  = 3
local CH_GEAR = 4
local CH_SLC  = 5
local CH_BODY = 6

-- ---------------------------------------------------------------------------
-- ПЕРЕДАЧІ  (незалежні від логіки газу — лише жорсткий ліміт PWM)
-- ---------------------------------------------------------------------------
-- Передача 1: макс 1200 мкс  |  Передача 2: макс 1350 мкс  |  Передача 3: макс 1600 мкс
local GEAR_MAX_PWM = { 1200, 1350, 1600 }
local nGear        = 1
local gear_max     = GEAR_MAX_PWM[1]

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
        nGear     = new_gear
        gear_max  = GEAR_MAX_PWM[nGear]
        log(SEV_INFO, string.format("Gear -> %d (max_pwm=%d)", nGear, gear_max))
    end
end

-- ---------------------------------------------------------------------------
-- ЧАСОВІ КОНСТАНТИ (мс)
-- ---------------------------------------------------------------------------
local RELAY_HOLD_MS     = 1000  -- скільки тримати ch3/ch5 перед спрацюванням реле
local BRAKE_HOLD_MS     = 350   -- час повного гальмування перед початком пульсації
local PULSE_RELEASE_MS  = 400   -- тривалість фази ВІДПУСКАННЯ при пульсації
local PULSE_HOLD_MS     = 80    -- тривалість фази ГАЛЬМУВАННЯ при пульсації

-- ---------------------------------------------------------------------------
-- СТАН РЕЛЕ БАТАРЕЇ
-- ---------------------------------------------------------------------------
local btr_timer = 0
local btr_armed = false
local btr_state = false  -- актуальний логічний стан реле батареї

local function update_battery(v_bt, now)
    if v_bt > 1100 then
        if not btr_armed then
            btr_timer = now
            btr_armed = true
            log(SEV_INFO, string.format("Battery: arming (hold %dms)", RELAY_HOLD_MS))
        end
        if now - btr_timer >= RELAY_HOLD_MS then
            if not btr_state then
                log(SEV_INFO, "Battery relay ON")
                btr_state = true
            end
            relay_on(IDX_BTR)
        end
    else
        if btr_armed then
            log(SEV_INFO, "Battery: arm canceled")
        end
        btr_armed = false
        if btr_state then
            log(SEV_INFO, "Battery relay OFF")
            btr_state = false
        end
        relay_off(IDX_BTR)
    end
end

-- ---------------------------------------------------------------------------
-- СТАН СЕЛЕКТОРА ХОДУ
-- ---------------------------------------------------------------------------
local slc_timer = 0
local slc_armed = false
local slc_state = false  -- актуальний логічний стан реле селектора

local function update_selector(v_sl, now)
    if v_sl > 1100 then
        if not slc_armed then
            slc_timer = now
            slc_armed = true
            log(SEV_INFO, string.format("Selector: arming (hold %dms)", RELAY_HOLD_MS))
        end
        if now - slc_timer >= RELAY_HOLD_MS then
            if not slc_state then
                log(SEV_INFO, "Selector relay ON (reverse)")
                slc_state = true
            end
            relay_on(IDX_SLC)
        end
    else
        if slc_armed then
            log(SEV_INFO, "Selector: arm canceled")
        end
        slc_armed = false
        if slc_state then
            log(SEV_INFO, "Selector relay OFF (forward)")
            slc_state = false
        end
        relay_off(IDX_SLC)
    end
end

-- ---------------------------------------------------------------------------
-- ГІДРАВЛІЧНІ ГАЛЬМА
-- ---------------------------------------------------------------------------
-- Логіка:
--   1. Насос ON + клапани тиску ON + клапани скидання OFF → тиск наростає → гальмо
--   2. Через BRAKE_HOLD_MS починається пульсація клапанів скидання:
--      - PULSE_RELEASE_MS: клапан скидання ВІДКРИТО (тиск падає, гальмо слабшає)
--      - PULSE_HOLD_MS:    клапан скидання ЗАКРИТО (тиск тримається, гальмо)
--   3. Мета: запобігти перегріву гідравліки при тривалому гальмуванні
-- ---------------------------------------------------------------------------
local brake_left_on   = false  -- ліва сторона загальмована
local brake_right_on  = false  -- права сторона загальмована
local brake_start_ms  = 0      -- момент початку гальмування
local brake_pulse_ms  = 0      -- момент останнього перемикання пульсації
local brake_releasing = false  -- true = зараз фаза відпускання (клапан відкрито)

local function _set_brake_valves(left, right, releasing)
    -- releasing=true: відкриваємо клапани скидання (знімаємо тиск)
    -- releasing=false: закриваємо клапани скидання (тримаємо тиск = гальмо)
    if left  then
        if releasing then relay_on(IDX_LRV) else relay_off(IDX_LRV) end
    end
    if right then
        if releasing then relay_on(IDX_RRV) else relay_off(IDX_RRV) end
    end
end

local function do_brake(left, right, now)
    -- Перевіряємо зміну конфігурації (перехід між лівим/правим/обома)
    local config_changed = (left ~= brake_left_on) or (right ~= brake_right_on)

    if config_changed then
        -- Звільняємо попередню сторону якщо вона більше не потрібна
        if brake_left_on  and not left  then relay_on(IDX_LRV); relay_off(IDX_LPV) end
        if brake_right_on and not right then relay_on(IDX_RRV); relay_off(IDX_RPV) end
        -- Скидаємо таймер при зміні конфігурації
        brake_start_ms = now
        brake_pulse_ms = now
        brake_releasing = false
        brake_left_on  = left
        brake_right_on = right
        log(SEV_INFO, string.format("Brake config: L=%s R=%s (full %dms then pulse)",
            tostring(left), tostring(right), BRAKE_HOLD_MS))
    end

    -- Насос і клапани тиску
    relay_on(IDX_PUMP)
    if left  then relay_on(IDX_LPV) end
    if right then relay_on(IDX_RPV) end

    -- Початкова фаза — повне гальмування (клапани скидання закриті)
    if now - brake_start_ms < BRAKE_HOLD_MS then
        _set_brake_valves(left, right, false)
        return
    end

    -- Фаза пульсації
    local interval = brake_releasing and PULSE_RELEASE_MS or PULSE_HOLD_MS
    if now - brake_pulse_ms >= interval then
        brake_pulse_ms  = now
        brake_releasing = not brake_releasing
        _set_brake_valves(left, right, brake_releasing)
    end
end

-- ---------------------------------------------------------------------------
-- КУЗОВ САМОСКИДА  (CH6)
-- ---------------------------------------------------------------------------
-- ReleSsvl (RELAY8) перемикає гідравліку з режиму гальм на циліндр кузова.
-- LRV і RRV тут виконують іншу роль:
--   LRV ON = клапан подачі → тиск у циліндр → кузов ПІДНІМАЄТЬСЯ
--   RRV ON = клапан скидання → тиск з циліндра → кузов ОПУСКАЄТЬСЯ
-- Безпека: логіка кузова активна тільки при нульовому газі (dgstk == 0).
-- ---------------------------------------------------------------------------
local body_state = "STOP"  -- LOCKED|UP|DOWN|STOP

local function update_body(v_bd, dgstk)
    local new_state
    if dgstk ~= 0 then
        -- рух або гальмо — кузов не чіпаємо
        new_state = "LOCKED"
        relay_off(IDX_SSVL)
        relay_off(IDX_PUMP)
        relay_off(IDX_LRV)
        relay_off(IDX_RRV)
    elseif v_bd > 1600 then
        -- Підняти кузов
        new_state = "UP"
        relay_on(IDX_SSVL)   -- перемикаємо гідравліку на кузов
        relay_on(IDX_PUMP)   -- насос ON
        relay_on(IDX_LRV)    -- клапан подачі → тиск у циліндр
        relay_off(IDX_RRV)   -- клапан скидання закрито
    elseif v_bd < 1400 then
        -- Опустити кузов
        new_state = "DOWN"
        relay_on(IDX_SSVL)   -- перемикаємо гідравліку на кузов
        relay_on(IDX_PUMP)   -- насос ON
        relay_off(IDX_LRV)   -- клапан подачі закрито
        relay_on(IDX_RRV)    -- клапан скидання → тиск виходить → кузов вниз
    else
        -- Центр — зупинити, кузов тримається на місці
        new_state = "STOP"
        relay_off(IDX_SSVL)
        relay_off(IDX_PUMP)
        relay_off(IDX_LRV)
        relay_off(IDX_RRV)
    end

    if new_state ~= body_state then
        log(SEV_INFO, "Body: " .. new_state)
        body_state = new_state
    end
end

local function un_brake()
    if brake_left_on or brake_right_on then
        relay_on(IDX_LRV);  relay_on(IDX_RRV)   -- відкриваємо скидання
        relay_off(IDX_PUMP)
        relay_off(IDX_LPV); relay_off(IDX_RPV)
        brake_left_on   = false
        brake_right_on  = false
        brake_releasing = false
        brake_pulse_ms  = 0
        log(SEV_INFO, "Brake released")
    end
end

-- ---------------------------------------------------------------------------
-- ГОЛОВНИЙ ЦИКЛ
-- ---------------------------------------------------------------------------
local prev_mode = "INIT"  -- GAS|BRAKE|TURN_LEFT|TURN_RIGHT|IDLE

local function update()
    local now  = millis()

    -- Читаємо всі RC-канали (значення 1000..2000 мкс, або nil при відсутності сигналу)
    local v1   = rc:get_pwm(CH_VERT) or 1500
    local v2   = rc:get_pwm(CH_HORT) or 1500
    local v_bt = rc:get_pwm(CH_BTR)  or 0
    local v_gr = rc:get_pwm(CH_GEAR) or 1000
    local v_sl = rc:get_pwm(CH_SLC)  or 0
    local v_bd = rc:get_pwm(CH_BODY) or 1500

    -- Оновлення передачі
    update_gear(v_gr)

    -- Реле батареї та селектора
    update_battery(v_bt, now)
    update_selector(v_sl, now)

    -- -----------------------------------------------------------------------
    -- ГАЗ / НЕЙТРАЛЬ / ГАЛЬМО
    -- -----------------------------------------------------------------------
    local dgstk = 0  -- 0=нейтраль, 1=газ, 2=гальмо

    if v1 > 1500 and v1 < 2000 then
        -- Джойстик вперед: газ
        dgstk = 1
        local gas_pwm = math.min(math.floor(1000 + (v1 - 1500) / 500.0 * 1000), gear_max)
        SRV_Channels:set_output_pwm(70, gas_pwm)
        un_brake()

    elseif v1 < 1400 and v1 >= 1000 then
        -- Джойстик назад: гальмо
        dgstk = 2
        SRV_Channels:set_output_pwm(70, 1000)
        do_brake(true, true, now)

    else
        -- Нейтраль
        dgstk = 0
        if v2 > 1400 and v2 < 1600 then
            -- Кермо теж в центрі — вільний хід
            SRV_Channels:set_output_pwm(70, 1000)
            un_brake()
        end
    end

    -- -----------------------------------------------------------------------
    -- РОЗВОРОТИ НА МІСЦІ (тільки з нейтралі)
    -- -----------------------------------------------------------------------
    if dgstk == 0 then
        if v2 < 1380 and v2 >= 1000 then
            -- Поворот вліво: гальмуємо ліву сторону, газ на праву
            do_brake(true, false, now)
            local gas_pwm = math.min(math.floor(1000 + (1485 - v2) / 885.0 * 1000), gear_max)
            SRV_Channels:set_output_pwm(70, gas_pwm)

        elseif v2 > 1600 and v2 < 2000 then
            -- Поворот вправо: гальмуємо праву сторону, газ на ліву
            do_brake(false, true, now)
            local gas_pwm = math.min(math.floor(1000 + (v2 - 1500) / 500.0 * 1000), gear_max)
            SRV_Channels:set_output_pwm(70, gas_pwm)
        end
    end

    -- Кузов самоскида
    update_body(v_bd, dgstk)

    -- Лог поточного режиму руху (на переходах)
    local mode
    if dgstk == 1 then
        mode = "GAS"
    elseif dgstk == 2 then
        mode = "BRAKE"
    elseif v2 < 1380 and v2 >= 1000 then
        mode = "TURN_LEFT"
    elseif v2 > 1600 and v2 < 2000 then
        mode = "TURN_RIGHT"
    else
        mode = "IDLE"
    end
    if mode ~= prev_mode then
        log(SEV_INFO, string.format("Mode: %s (v1=%d v2=%d gear=%d)", mode, v1, v2, nGear))
        prev_mode = mode
    end

    return update, 20  -- наступний виклик через 20 мс (50 Гц)
end

-- ---------------------------------------------------------------------------
-- ІНІЦІАЛІЗАЦІЯ: всі реле вимкнути, газ на мінімум
-- ---------------------------------------------------------------------------
local function init()
    for i = 0, 7 do relay_off(i) end
    SRV_Channels:set_output_pwm(70, 1000)
    log(SEV_INFO, string.format("JRVS_MamontUGV started, gear=%d k=%.2f",
        nGear, GEAR_COEFF[nGear]))
    return update, 500  -- перший цикл через 500 мс після старту
end

return init()
