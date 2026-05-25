-- JKBMS debug: reads raw UART, prints hex frames to GCS Messages
-- Setup: SERIAL2_PROTOCOL=28, SCR_ENABLE=1, SCR_HEAP_SIZE>=65536
-- Copy to SD card: /APM/scripts/jkbms_debug.lua

local BAUD      = 2400
local HEADER_0  = 0xA5
local HEADER_1  = 0x5A

local port = serial:find_serial(0)
if not port then
    gcs:send_text(3, "JKBMS DBG: no scripting serial found")
    return
end

port:begin(BAUD)
port:set_flow_control(0)

local buf       = {}   -- raw byte accumulator
local total_rx  = 0    -- total bytes received since boot
local frames_ok = 0    -- valid frames detected

local function buf_to_hex(b, max_bytes)
    local s = {}
    local n = math.min(#b, max_bytes or #b)
    for i = 1, n do
        s[i] = string.format("%02X", b[i])
    end
    return table.concat(s, " ")
end

local function try_parse()
    -- find header
    while #buf >= 2 do
        if buf[1] == HEADER_0 and buf[2] == HEADER_1 then
            break
        end
        table.remove(buf, 1)
    end

    if #buf < 3 then return end

    local data_len   = buf[3]
    local frame_total = data_len + 3

    if frame_total > 200 then
        -- corrupt length, skip one byte
        table.remove(buf, 1)
        return
    end

    if #buf < frame_total then return end  -- wait for more bytes

    frames_ok = frames_ok + 1
    local ftype = buf[4] or 0

    -- print first 16 bytes of frame as hex
    local hex = buf_to_hex(buf, math.min(frame_total, 16))
    gcs:send_text(6, string.format(
        "JKBMS frame#%d type=0x%02X len=%d | %s%s",
        frames_ok, ftype, frame_total, hex,
        frame_total > 16 and "..." or ""))

    -- consume the frame
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

    -- every ~10 s print a heartbeat so we know script is alive
    if math.fmod(total_rx, 240) == 1 then
        gcs:send_text(6, string.format(
            "JKBMS DBG: rx=%d bytes, frames=%d, buf=%d",
            total_rx, frames_ok, #buf))
    end

    return update, 100   -- run every 100 ms
end

gcs:send_text(6, string.format("JKBMS DBG: started, baud=%d", BAUD))
return update, 1000
