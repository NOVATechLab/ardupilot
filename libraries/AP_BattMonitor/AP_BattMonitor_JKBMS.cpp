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

/*
 * JKONG BMS — GPS/LCD port, LCD Protocol V2.0
 *
 * Protocol (active push, no request needed, no checksum):
 *   Frame: [0xA5][0x5A][data_len][frame_type][data_len bytes payload]
 *   frame_total_bytes = data_len + 3
 *   Baud: 115200
 *
 * Frame types observed:
 *
 *   0x80, data_len=3  — Handshake / heartbeat. Ignored.
 *
 *   0x82, data[4..5] = 0x10 0x00 (reg block 0x1000), data_len=45 — Basic status:
 *     data[6..7]   reg 0x1000  Total voltage  UINT16 x0.01V
 *     data[8..9]   reg 0x1001  Current        INT16  x0.1A  (positive=charging)
 *     data[12..13] reg 0x1003  SOC            UINT16 %
 *     data[16..17] reg 0x1005  MOSFET temp    INT16  degC
 *     data[18..19] reg 0x1006  Battery temp   INT16  degC  (-200 = sensor absent)
 *
 *   0x82, data[4..5] = 0x11 0x00 (reg block 0x1100), data_len=59 — Cell voltages:
 *     data[6..7]   cell 1 voltage  UINT16 mV
 *     data[8..9]   cell 2 voltage  UINT16 mV
 *     ...up to 28 cells; inactive cells = 0x0000
 *
 * Reference:
 *   https://github.com/syssi/esphome-jk-bms/tree/main/components/jk_bms_display
 */

#include "AP_BattMonitor_JKBMS.h"

#if AP_BATTERY_JKBMS_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

// Register block identifiers in data[4..5]
static const uint8_t REG_BLOCK_STATUS_HI   = 0x10;  // Basic status registers
static const uint8_t REG_BLOCK_STATUS_LO   = 0x00;
static const uint8_t REG_BLOCK_CELLS_HI    = 0x11;  // Cell voltage registers
static const uint8_t REG_BLOCK_CELLS_LO    = 0x00;

// Expected data_len for each block
static const uint8_t DATA_LEN_STATUS = 45;
static const uint8_t DATA_LEN_CELLS  = 59;

// Maximum cells in a cell-block frame (28 slots, only active ones are non-zero)
static const uint8_t CELLS_PER_FRAME = 28;

void AP_BattMonitor_JKBMS::init()
{
    _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_JKBMS, 0);
    if (_uart != nullptr) {
        _uart->begin(AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_JKBMS, 0));
    }
}

void AP_BattMonitor_JKBMS::read()
{
    if (_uart == nullptr) {
        return;
    }

    uint32_t nbytes = _uart->available();
    while (nbytes-- > 0 && _rx_len < RX_BUF_SIZE) {
        _rx_buf[_rx_len++] = _uart->read();
    }

    _process_bytes();

    const uint32_t now_ms = AP_HAL::millis();
    if (_got_data && (now_ms - _last_response_ms > HEALTHY_TIMEOUT_MS)) {
        _state.healthy = false;
    }
}

bool AP_BattMonitor_JKBMS::capacity_remaining_pct(uint8_t &percentage) const
{
    if (!_got_data) {
        return false;
    }
    percentage = _soc_pct;
    return true;
}

void AP_BattMonitor_JKBMS::_consume_bytes(uint16_t n)
{
    if (n >= _rx_len) {
        _rx_len = 0;
        return;
    }
    memmove(_rx_buf, _rx_buf + n, _rx_len - n);
    _rx_len -= n;
}

void AP_BattMonitor_JKBMS::_process_bytes()
{
    // Discard leading garbage before 0xA5 0x5A
    while (_rx_len >= 2) {
        if (_rx_buf[0] == FRAME_HEADER_0 && _rx_buf[1] == FRAME_HEADER_1) {
            break;
        }
        _consume_bytes(1);
    }

    if (_rx_len < 3) {
        return;
    }

    const uint8_t  data_len    = _rx_buf[2];
    const uint16_t frame_total = (uint16_t)data_len + 3;

    if (frame_total > RX_BUF_SIZE) {
        _consume_bytes(1);  // corrupt length byte
        return;
    }

    if (_rx_len < frame_total) {
        return;  // wait for more bytes
    }

    // No checksum in this protocol — process immediately
    _parse_frame(frame_total);
    _consume_bytes(frame_total);
}

void AP_BattMonitor_JKBMS::_parse_frame(uint16_t frame_total_len)
{
    const uint8_t *raw     = _rx_buf;
    const uint8_t  data_len = raw[2];
    const uint8_t  frame_type = raw[3];

    auto get16u = [&](uint16_t pos) -> uint16_t {
        return ((uint16_t)raw[pos] << 8) | raw[pos + 1];
    };

    if (frame_type == 0x80) {
        return;  // handshake, nothing to do
    }

    if (frame_type != 0x82) {
        return;  // unknown frame type
    }

    // Need at least the register-block identifier bytes (data[4..5])
    if (data_len < 3) {
        return;
    }

    const uint8_t reg_hi = raw[4];
    const uint8_t reg_lo = raw[5];

    // ---------------------------------------------------------------
    // Basic status frame (reg block 0x1000)
    // ---------------------------------------------------------------
    if (reg_hi == REG_BLOCK_STATUS_HI && reg_lo == REG_BLOCK_STATUS_LO
            && data_len >= DATA_LEN_STATUS) {

        // Total voltage: reg 0x1000, data[6..7], x0.01V
        const float voltage = get16u(6) * 0.01f;

        // Current: reg 0x1001, data[8..9], INT16 x0.1A
        // JK convention: positive=charging; ArduPilot: positive=discharging -> negate
        const float current_amps = -(int16_t)get16u(8) * 0.1f;

        // SOC: reg 0x1003, data[12..13], %
        _soc_pct = (uint8_t)MIN((uint16_t)100, get16u(12));

        // Temperatures: INT16 degC
        // reg 0x1005 = MOSFET temp, data[16..17]
        // reg 0x1006 = battery temp, data[18..19]; -200 means sensor absent
        const int16_t mosfet_temp  = (int16_t)get16u(16);
        const int16_t battery_temp = (int16_t)get16u(18);
        const bool    batt_ok      = (battery_temp > -100 && battery_temp < 150);
        const float   temperature  = batt_ok ? (float)battery_temp : (float)mosfet_temp;

        const uint32_t now_us = AP_HAL::micros();
        const uint32_t dt_us  = now_us - _state.last_time_micros;

        _state.voltage          = voltage;
        _state.current_amps     = current_amps;
        _state.temperature      = temperature;
        _state.last_time_micros = now_us;
        _state.healthy          = true;

        if (_got_data && dt_us > 0) {
            update_consumed(_state, dt_us);
        }

        _got_data         = true;
        _last_response_ms = AP_HAL::millis();
        return;
    }

    // ---------------------------------------------------------------
    // Cell voltage frame (reg block 0x1100)
    // ---------------------------------------------------------------
    if (reg_hi == REG_BLOCK_CELLS_HI && reg_lo == REG_BLOCK_CELLS_LO
            && data_len >= DATA_LEN_CELLS) {

        // Cells start at raw[6]; inactive cells = 0x0000
        const uint8_t max_cells = MIN((uint8_t)CELLS_PER_FRAME,
                                      (uint8_t)AP_BATT_MONITOR_CELLS_MAX);
        uint8_t active = 0;
        for (uint8_t i = 0; i < max_cells; i++) {
            const uint16_t mv = get16u(6 + i * 2);
            _state.cell_voltages.cells[i] = (mv > 0) ? mv : UINT16_MAX;
            if (mv > 0) {
                active++;
            }
        }
        // Invalidate remaining slots
        for (uint8_t i = max_cells; i < AP_BATT_MONITOR_CELLS_MAX; i++) {
            _state.cell_voltages.cells[i] = UINT16_MAX;
        }

        if (active > 0) {
            _has_cell_voltages = true;
        }
        return;
    }

    // All other frames (small 5-byte config frames, etc.) are silently ignored
}

#endif  // AP_BATTERY_JKBMS_ENABLED
