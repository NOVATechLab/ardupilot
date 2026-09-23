-- JKBMS debug: scans all scripting UART ports, reports which one gets data
-- Setup: set SERIAL1_PROTOCOL=28, SERIAL2_PROTOCOL=28, SERIAL3_PROTOCOL=28
--        SCR_ENABLE=1, then reboot

local BAUD       = 115200
local MAX_PORTS  = 4

local ports = {}
for i = 0, MAX_PORTS - 1 do
    local p = serial:find_serial(i)
    if p then
        p:begin(BAUD)
        p:set_flow_control(0)
        ports[i] = { port = p, rx = 0, last_byte = -1 }
        gcs:send_text(6, string.format("JKBMS DBG: found scripting port #%d", i))
    end
end

if #ports == 0 and ports[0] == nil then
    gcs:send_text(3, "JKBMS DBG: no scripting ports found! Set SERIALx_PROTOCOL=28")
    return
end

local last_hb = millis()

function update()
    -- read all ports
    for i = 0, MAX_PORTS - 1 do
        local entry = ports[i]
        if entry then
            local n = tonumber(entry.port:available()) or 0
            if n > 0 then
                for _ = 1, n do
                    local b = tonumber(entry.port:read()) or -1
                    if b >= 0 then
                        entry.rx = entry.rx + 1
                        entry.last_byte = b
                    end
                end
            end
        end
    end

    -- heartbeat every 5 s
    local now = millis()
    if now - last_hb >= 5000 then
        last_hb = now
        for i = 0, MAX_PORTS - 1 do
            local entry = ports[i]
            if entry then
                gcs:send_text(6, string.format(
                    "JKBMS port#%d: rx=%d bytes  last=0x%02X",
                    i, entry.rx,
                    entry.last_byte >= 0 and entry.last_byte or 0))
            end
        end
    end

    return update, 100
end

gcs:send_text(6, string.format("JKBMS DBG: scanning %d port(s) at %d baud", MAX_PORTS, BAUD))
return update, 1000
