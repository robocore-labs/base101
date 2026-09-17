#include "st3215.h"

#include <string.h>

#include "pico/stdlib.h"

#include "../bus/half_duplex.h"

#define ST_INSTR_PING  0x01
#define ST_INSTR_READ  0x02
#define ST_INSTR_WRITE 0x03

#define ST_RESPONSE_TIMEOUT_US 2000
#define ST_RETRIES 1

// Feetech STS multi-byte registers are little-endian; signed values use
// a sign-magnitude encoding with the sign in the top bit of the field.
static int16_t decode_signed(uint16_t raw, uint16_t sign_bit) {
    int16_t mag = (int16_t)(raw & (sign_bit - 1));
    return (raw & sign_bit) ? (int16_t)-mag : mag;
}

static uint8_t checksum(const uint8_t *pkt, uint32_t len) {
    // sum over id..params (skip the two 0xFF header bytes)
    uint32_t sum = 0;
    for (uint32_t i = 2; i < len; i++) {
        sum += pkt[i];
    }
    return (uint8_t)(~sum);
}

// Send instruction packet, receive status packet.
// rx_params_len = expected parameter byte count of the status packet.
// Status packet: [0xFF 0xFF id len error params... checksum], len = params + 2.
static bool st_transact(uint8_t id, uint8_t instr, const uint8_t *params, uint8_t params_len,
                        uint8_t *rx_params, uint8_t rx_params_len) {
    uint8_t tx[32];
    uint32_t tx_len = 0;
    tx[tx_len++] = 0xFF;
    tx[tx_len++] = 0xFF;
    tx[tx_len++] = id;
    tx[tx_len++] = (uint8_t)(params_len + 2);
    tx[tx_len++] = instr;
    for (uint8_t i = 0; i < params_len; i++) {
        tx[tx_len++] = params[i];
    }
    tx[tx_len] = checksum(tx, tx_len);
    tx_len++;

    uint8_t rx[32];
    uint32_t rx_len = 6u + rx_params_len;

    for (int attempt = 0; attempt <= ST_RETRIES; attempt++) {
        int got = half_duplex_transact(tx, tx_len, rx, rx_len, ST_RESPONSE_TIMEOUT_US);
        if (got != (int)rx_len) {
            continue;
        }
        if (rx[0] != 0xFF || rx[1] != 0xFF || rx[2] != id) {
            continue;
        }
        if (checksum(rx, rx_len - 1) != rx[rx_len - 1]) {
            continue;
        }
        // rx[4] is the servo error byte; tolerate non-zero (overload bits
        // etc.) like the python library does — data is still valid.
        if (rx_params != NULL && rx_params_len > 0) {
            memcpy(rx_params, &rx[5], rx_params_len);
        }
        return true;
    }
    return false;
}

static bool st_write(uint8_t id, uint8_t reg, const uint8_t *data, uint8_t len) {
    uint8_t params[16];
    params[0] = reg;
    memcpy(&params[1], data, len);
    return st_transact(id, ST_INSTR_WRITE, params, (uint8_t)(len + 1), NULL, 0);
}

static bool st_write_u8(uint8_t id, uint8_t reg, uint8_t value) {
    return st_write(id, reg, &value, 1);
}

static bool st_read(uint8_t id, uint8_t reg, uint8_t *data, uint8_t len) {
    uint8_t params[2] = {reg, len};
    return st_transact(id, ST_INSTR_READ, params, 2, data, len);
}

// ---------------------------------------------------------------------------

bool st3215_ping(uint8_t id) {
    return st_transact(id, ST_INSTR_PING, NULL, 0, NULL, 0);
}

bool st3215_set_id(uint8_t old_id, uint8_t new_id) {
    st_write_u8(old_id, ST3215_REG_LOCK, 0);   // unlock EEPROM
    sleep_ms(5);
    if (!st_write_u8(old_id, ST3215_REG_ID, new_id)) {
        return false;
    }
    sleep_ms(10);  // let the EEPROM write settle; servo now answers at new_id
    st_write_u8(new_id, ST3215_REG_LOCK, 1);   // re-lock EEPROM
    return st3215_ping(new_id);
}

bool st3215_set_mode(uint8_t id, uint8_t mode) {
    return st_write_u8(id, ST3215_REG_MODE, mode);
}

bool st3215_set_torque(uint8_t id, bool enable) {
    return st_write_u8(id, ST3215_REG_TORQUE_ENABLE, enable ? 1 : 0);
}

bool st3215_set_acceleration(uint8_t id, uint8_t accel) {
    return st_write_u8(id, ST3215_REG_GOAL_ACC, accel);
}

bool st3215_move_to(uint8_t id, uint16_t position, uint16_t speed, uint8_t accel) {
    if (position > 4095) position = 4095;
    if (!st_write_u8(id, ST3215_REG_GOAL_ACC, accel)) {
        return false;
    }
    // Goal position, time (0) and speed in one write starting at reg 42.
    uint8_t data[6] = {
        (uint8_t)(position & 0xFF), (uint8_t)(position >> 8),
        0x00, 0x00,
        (uint8_t)(speed & 0xFF), (uint8_t)(speed >> 8),
    };
    return st_write(id, ST3215_REG_GOAL_POSITION_L, data, sizeof(data));
}

bool st3215_rotate(uint8_t id, int16_t speed) {
    if (speed > ST3215_MAX_SPEED) speed = ST3215_MAX_SPEED;
    if (speed < -ST3215_MAX_SPEED) speed = -ST3215_MAX_SPEED;

    uint16_t raw;
    if (speed < 0) {
        raw = (uint16_t)(-speed) | 0x8000;
    } else {
        raw = (uint16_t)speed;
    }
    uint8_t data[2] = {(uint8_t)(raw & 0xFF), (uint8_t)(raw >> 8)};
    return st_write(id, ST3215_REG_GOAL_SPEED_L, data, sizeof(data));
}

int32_t st3215_read_position(uint8_t id) {
    uint8_t data[2];
    if (!st_read(id, ST3215_REG_PRESENT_POS_L, data, 2)) {
        return -1;
    }
    return (int32_t)(((uint16_t)data[1] << 8) | data[0]);
}

bool st3215_read_speed(uint8_t id, int16_t *speed) {
    uint8_t data[2];
    if (!st_read(id, ST3215_REG_PRESENT_SPEED_L, data, 2)) {
        return false;
    }
    *speed = decode_signed((uint16_t)(((uint16_t)data[1] << 8) | data[0]), 0x8000);
    return true;
}

bool st3215_read_state(uint8_t id, uint16_t *position, int16_t *speed) {
    uint8_t data[4];
    if (!st_read(id, ST3215_REG_PRESENT_POS_L, data, 4)) {
        return false;
    }
    *position = (uint16_t)(((uint16_t)data[1] << 8) | data[0]);
    *speed = decode_signed((uint16_t)(((uint16_t)data[3] << 8) | data[2]), 0x8000);
    return true;
}

bool st3215_read_telemetry(uint8_t id, st3215_telemetry_t *t) {
    // Load (60-61), voltage (62), temperature (63) in one read.
    uint8_t lvt[4];
    if (!st_read(id, ST3215_REG_PRESENT_LOAD_L, lvt, 4)) {
        return false;
    }
    uint8_t cur[2];
    if (!st_read(id, ST3215_REG_PRESENT_CURRENT, cur, 2)) {
        return false;
    }

    uint16_t load_raw = (uint16_t)(((uint16_t)lvt[1] << 8) | lvt[0]);
    // Load: 0..1000 = 0..100%, sign in bit 10.
    t->load_pct = (float)decode_signed(load_raw, 0x0400) / 10.0f;
    t->voltage_v = (float)lvt[2] / 10.0f;
    t->temperature_c = lvt[3];

    uint16_t cur_raw = (uint16_t)(((uint16_t)cur[1] << 8) | cur[0]);
    // Current unit: 6.5 mA per LSB (STS3215 datasheet).
    t->current_ma = (float)decode_signed(cur_raw, 0x8000) * 6.5f;
    return true;
}
