-- =============================================================================
-- MAUL_CanDecode.lua — passive Can-Am/BRP CAN bus decoder for the UGV_Maul rover
-- =============================================================================
-- Purely passive listener. Never writes to the bus. Decodes the broadcast frames
-- documented in debug/canam_can_map.md and publishes them into the global table
-- _G.MAUL_CAN so MAUL_Control.lua can read them (gear feedback + speed feed the
-- gear-change safety interlock; the rest is telemetry).
--
-- Requires one CAN port set to CAN_Dx_PROTOCOL=10 (Scripting), bitrate 500000,
-- wired to the vehicle's OEM CAN bus.
-- =============================================================================

local SEV_INFO = 6
local SEV_WARN = 4
local function log(sev, msg) gcs:send_text(sev, "MAUL_CAN: " .. msg) end

local driver = CAN:get_device(25)
if not driver then
    log(SEV_WARN, "no scripting CAN driver found (check CAN_Dx_PROTOCOL=10)")
    return
end

-- ---------------------------------------------------------------------------
-- Shared output table — MAUL_Control.lua reads this. ts_* fields are millis()
-- timestamps of the last accepted frame for that message, for staleness checks.
-- ---------------------------------------------------------------------------
_G.MAUL_CAN = {
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
    _G.MAUL_CAN.pedal_pct     = frame:data(0) * 100.0 / 254.0
    _G.MAUL_CAN.pedal_pressed = (frame:data(3) & 0x40) ~= 0
    _G.MAUL_CAN.shaft_locked  = (frame:data(3) & 0x01) ~= 0
    _G.MAUL_CAN.ts_103        = now
end

local function handle_gear(frame, now)
    if not checksum_ok(frame) then return end
    local b0, b1 = frame:data(0), frame:data(1)
    local gear = GEAR_MAP[b0]
    _G.MAUL_CAN.gear_valid = (b0 == b1) and (gear ~= nil)
    _G.MAUL_CAN.gear       = _G.MAUL_CAN.gear_valid and gear or "INVALID"
    _G.MAUL_CAN.ts_309     = now
end

local function handle_shaftlock2(frame, now)
    if not checksum_ok(frame) then return end
    -- duplicate of the 0x103 lock bit, kept only as a cross-check / future use
    _G.MAUL_CAN.shaft_locked = (frame:data(0) & 0x01) ~= 0
    _G.MAUL_CAN.ts_103       = now
end

local function handle_drivemode(frame, now)
    _G.MAUL_CAN.drive_4wd = (frame:data(5) & 0x10) ~= 0
    _G.MAUL_CAN.ts_400    = now
end

local function handle_rpm(frame, now)
    _G.MAUL_CAN.rpm         = (frame:data(2) << 8) | frame:data(3)
    _G.MAUL_CAN.engine_load = frame:data(5)
    _G.MAUL_CAN.ts_102      = now
end

local function handle_speed(frame, now)
    _G.MAUL_CAN.speed_kmh = ((frame:data(0) << 8) | frame:data(1)) / 10.0
    _G.MAUL_CAN.ts_231    = now
end

local function handle_fuel(frame, now)
    local raw = frame:data(4)
    _G.MAUL_CAN.fuel_sensor_open = (raw == 0x7F)
    if raw ~= 0x7F then
        _G.MAUL_CAN.fuel_pct = raw
    end
    _G.MAUL_CAN.ts_530 = now
end

local function handle_fuelfault(frame, now)
    _G.MAUL_CAN.fuel_fault   = not (frame:data(2) == 0x99 and frame:data(3) == 0x99)
    _G.MAUL_CAN.batt_voltage = frame:data(4) * 0.1
    _G.MAUL_CAN.ts_342       = now
end

local function update()
    local now = millis()

    while true do
        local frame = driver:read_frame()
        if not frame then break end

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

    return update, 10
end

log(SEV_INFO, "ready")
return update, 1000
