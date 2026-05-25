-- JKBMS debug: reads raw UART, prints hex frames to GCS Messages
-- Setup: SERIAL2_PROTOCOL=28, SCR_ENABLE=1

local BAUD        = 2400
local HEADER_0    = 0xA5
local HEADER_1    = 0x5A
local HEARTBEAT_MS = 5000

local port = serial:find_serial(0)
if not port then
    gcs:send_text(3, "JKBMS DBG: no scripting serial found")
    return
end

port:begin(BAUD)
port:set_flow_control(0)

local buf        = {}
local total_rx   = 0
local frames_ok  = 0
local last_hb_ms = 0

local function buf_to_hex(b, max_bytes)
    local s = {}
    local n = math.min(#b, max_bytes or #b)
    for i = 1, n do
        s[i] = string.format("%02X", b[i])
    end
    return table.concat(s, " ")
end

local function try_parse()
    while #buf >= 2 do
        if buf[1] == HEADER_0 and buf[2] == HEADER_1 then break end
        table.remove(buf, 1)
    end

    if #buf < 3 then return end

    local data_len    = buf[3]
    local frame_total = data_len + 3

    if frame_total > 200 then
        table.remove(buf, 1)
        return
    end

    if #buf < frame_total then return end

    frames_ok = frames_ok + 1
    local ftype = buf[4] or 0
    local hex   = buf_to_hex(buf, math.min(frame_total, 16))
    gcs:send_text(6, string.format(
        "JKBMS frame#%d type=0x%02X len=%d | %s%s",
        frames_ok, ftype, frame_total, hex,
        frame_total > 16 and "..." or ""))

    for _ = 1, frame_total do
        table.remove(buf, 1)
    end
end

function update()
    local n = tonumber(port:available()) or 0
    if n > 0 then
        for _ = 1, n do
            local b = tonumber(port:read()) or -1
            if b >= 0 then
                buf[#buf + 1] = b
                total_rx = total_rx + 1
            end
        end
        try_parse()
    end

    -- time-based heartbeat every 5 s regardless of byte count
    local now = millis()
    if tonumber(now) - last_hb_ms >= HEARTBEAT_MS then
        last_hb_ms = tonumber(now)
        gcs:send_text(6, string.format(
            "JKBMS DBG: rx=%d bytes  frames=%d  buf=%d",
            total_rx, frames_ok, #buf))
    end

    return update, 100
end

gcs:send_text(6, string.format("JKBMS DBG: started, baud=%d", BAUD))
return update, 1000
