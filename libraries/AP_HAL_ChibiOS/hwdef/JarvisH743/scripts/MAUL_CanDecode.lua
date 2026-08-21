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
--
-- Bring-up logging (see debug/MAUL_UGV_README.md): a "first frame seen"
-- message logs once per known CAN ID the first time it's actually observed
-- on the bus, and a "no CAN frames received yet" warning repeats every 5s
-- until something arrives -- both always on. MAULCAN_DBG=1 (default) also
-- logs a periodic one-line summary of the decoded values every
-- MAULCAN_DBG_MS; set to 0 once you're done bench-testing.
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
-- BRING-UP DEBUG LOGGING (table 74, prefix MAULCAN_)
-- Always: logs once, the first time each known CAN ID is actually seen on
-- the bus (fast way to confirm wiring/IDs during bench testing), and warns
-- periodically if nothing has arrived at all. Optionally (MAULCAN_DBG=1):
-- a periodic one-line summary of the decoded values in GCS messages.
-- ---------------------------------------------------------------------------
local PARAM_TABLE_KEY    = 74
local PARAM_TABLE_PREFIX = "MAULCAN_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 2), "MAUL_CAN: add_table failed")

local function add_param(idx, name, default)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default), "MAUL_CAN: add_param " .. name)
    local p = Parameter()
    assert(p:init(PARAM_TABLE_PREFIX .. name), "MAUL_CAN: init " .. name)
    return p
end

local P_DBG    = add_param(1, 'DBG',    1)    -- 0=off, 1=periodic decoded summary in GCS messages
local P_DBG_MS = add_param(2, 'DBG_MS', 2000) -- ms between summary logs when DBG=1

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
local seen_ids     = {}
local unseen_count = #KNOWN_IDS

local total_frames        = 0
local last_summary_ms     = 0
local last_nodata_warn_ms = 0

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

local function note_first_seen(frame)
    if unseen_count == 0 then return end
    local id_num = frame:id():toint()
    for _, k in ipairs(KNOWN_IDS) do
        if k.id == id_num and not seen_ids[k.id] then
            seen_ids[k.id] = true
            unseen_count = unseen_count - 1
            log(SEV_INFO, "first frame seen: " .. k.name)
            break
        end
    end
end

local function update()
    local now = millis()

    while true do
        local frame = driver:read_frame()
        if not frame then break end

        total_frames = total_frames + 1
        note_first_seen(frame)

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

    if total_frames == 0 then
        if now - last_nodata_warn_ms > 5000 then
            log(SEV_WARN, "no CAN frames received yet - check wiring/CAN_Dx_PROTOCOL/bitrate")
            last_nodata_warn_ms = now
        end
    elseif P_DBG:get() ~= 0 then
        local dbg_ms = P_DBG_MS:get()
        if now - last_summary_ms > dbg_ms then
            last_summary_ms = now
            log(SEV_INFO, string.format(
                "frames=%d gear=%s(%s) spd=%.1f rpm=%d pedal=%.0f%% batt=%.1fV",
                total_frames,
                _G.MAUL_CAN.gear,
                _G.MAUL_CAN.gear_valid and "ok" or "inv",
                _G.MAUL_CAN.speed_kmh,
                _G.MAUL_CAN.rpm,
                _G.MAUL_CAN.pedal_pct,
                _G.MAUL_CAN.batt_voltage))
        end
    end

    return update, 10
end

log(SEV_INFO, "ready")
return update, 1000
