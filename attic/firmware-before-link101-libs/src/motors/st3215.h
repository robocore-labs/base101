/*
 * Feetech ST3215 (STS series) servo driver on the Axon half-duplex
 * motor bus (PIO0, 1 Mbaud).
 *
 * Firmware port of the parts of the `st3215` python library used by
 * LLMy st3215_manager: ping, mode select, torque control, position
 * moves, wheel-mode rotation and telemetry readback.
 *
 * Packet format (Feetech SCS/STS protocol, same framing as Dynamixel 1.0):
 *   [0xFF 0xFF id len instruction params... checksum]
 *   checksum = ~(id + len + instruction + sum(params)) & 0xFF
 */

#ifndef MOTORS_ST3215_H
#define MOTORS_ST3215_H

#include <stdbool.h>
#include <stdint.h>

// Control table (STS3215)
#define ST3215_REG_ID               5  // bus ID (EEPROM, lock-protected)
#define ST3215_REG_MODE            33  // 0 = position, 1 = wheel/velocity
#define ST3215_REG_TORQUE_ENABLE   40
#define ST3215_REG_GOAL_ACC        41
#define ST3215_REG_GOAL_POSITION_L 42
#define ST3215_REG_GOAL_TIME_L     44
#define ST3215_REG_GOAL_SPEED_L    46
#define ST3215_REG_LOCK            55
#define ST3215_REG_PRESENT_POS_L   56
#define ST3215_REG_PRESENT_SPEED_L 58
#define ST3215_REG_PRESENT_LOAD_L  60
#define ST3215_REG_PRESENT_VOLTAGE 62
#define ST3215_REG_PRESENT_TEMP    63
#define ST3215_REG_PRESENT_CURRENT 69

#define ST3215_MODE_POSITION 0
#define ST3215_MODE_VELOCITY 1

#define ST3215_MAX_SPEED 3400  // steps/s

typedef struct {
    float   current_ma;
    float   voltage_v;
    float   load_pct;     // signed, -100..100
    int16_t temperature_c;
} st3215_telemetry_t;

// Returns true if the servo answered the ping.
bool st3215_ping(uint8_t id);

// Change a servo's bus ID (EEPROM): unlock, write reg 5, re-lock. The servo
// must be the only one on the bus at its current id. Returns true if the
// servo answers at new_id afterwards.
bool st3215_set_id(uint8_t old_id, uint8_t new_id);

bool st3215_set_mode(uint8_t id, uint8_t mode);
bool st3215_set_torque(uint8_t id, bool enable);
bool st3215_set_acceleration(uint8_t id, uint8_t accel);

// Position move: goal position in ticks (0..4095), speed in steps/s,
// accel in register units (0-255).
bool st3215_move_to(uint8_t id, uint16_t position, uint16_t speed, uint8_t accel);

// Wheel-mode rotation, speed in steps/s, signed.
bool st3215_rotate(uint8_t id, int16_t speed);

// Present position in ticks. Returns -1 on failure.
int32_t st3215_read_position(uint8_t id);

// Present speed in steps/s (signed). Returns true on success.
bool st3215_read_speed(uint8_t id, int16_t *speed);

// Position + speed in one bus transaction (registers 56..59).
bool st3215_read_state(uint8_t id, uint16_t *position, int16_t *speed);

// Current/voltage/load/temperature. Returns true if all reads succeeded.
bool st3215_read_telemetry(uint8_t id, st3215_telemetry_t *t);

#endif // MOTORS_ST3215_H
