/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "AP_BattMonitor_config.h"

#if AP_BATTERY_JKBMS_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_BattMonitor_Backend.h"

// JKONG BMS — GPS/LCD port, LCD Protocol V2.0
// Active push: BMS sends data every ~1 s; no request frame needed.
// Default baud rate: 2400 (set SERIALX_PROTOCOL=49, BATT_MONITOR=30).
class AP_BattMonitor_JKBMS : public AP_BattMonitor_Backend
{
public:
    using AP_BattMonitor_Backend::AP_BattMonitor_Backend;

    void init() override;
    void read() override;

    bool has_current() const override { return true; }
    bool has_cell_voltages() const override { return _has_cell_voltages; }
    bool has_temperature() const override { return _got_data; }

    bool capacity_remaining_pct(uint8_t &percentage) const override WARN_IF_UNUSED;

private:
    AP_HAL::UARTDriver *_uart = nullptr;

    static const uint32_t HEALTHY_TIMEOUT_MS = 5000;
    static const uint16_t RX_BUF_SIZE        = 256;  // max frame ~96 bytes

    // LCD V2.0 frame header bytes
    static const uint8_t FRAME_HEADER_0 = 0xA5;
    static const uint8_t FRAME_HEADER_1 = 0x5A;

    uint8_t  _rx_buf[RX_BUF_SIZE];
    uint16_t _rx_len = 0;

    uint32_t _last_response_ms = 0;
    bool     _got_data         = false;
    bool     _has_cell_voltages = false;
    uint8_t  _soc_pct          = 0;

    void _process_bytes();
    void _parse_frame(uint16_t frame_total_len);
    void _consume_bytes(uint16_t n);
};

#endif  // AP_BATTERY_JKBMS_ENABLED
