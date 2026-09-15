/*
 * Waveshare DDSM210 direct-drive servo motor driver.
 *
 * Firmware port of LLMy ddsm210_manager/ddsm210.py.
 *
 * Protocol: 115200 baud 8N1, 10-byte frames, CRC-8/MAXIM.
 *   [ID, CMD, D2..D8, CRC8]
 * Commands:
 *   0x64 drive (velocity or position depending on mode)
 *   0x74 get mileage/position feedback
 *   0x75 query mode
 *   0xA0 set mode (0x00 open loop, 0x02 velocity, 0x03 position)
 *
 * One motor per UART channel (shared TX lines are not supported by the
 * motor without external gating).
 */

#ifndef MOTORS_DDSM210_H
#define MOTORS_DDSM210_H

#include <stdbool.h>
#include <stdint.h>

#include "../bus/ddsm_port.h"

#define DDSM210_MODE_OPEN_LOOP 0x00
#define DDSM210_MODE_VELOCITY  0x02
#define DDSM210_MODE_POSITION  0x03

#define DDSM210_MAX_SPEED_RAW  2100   // 0.1 RPM units = 210 RPM
#define DDSM210_MAX_POSITION   32767  // = 360 degrees
#define DDSM210_ENCODER_TICKS  65536  // 0..65535 = 0..360 degrees

typedef struct {
    int16_t feedback1;    // speed (0.1 RPM) by default
    int16_t feedback2;    // position by default
    uint8_t accel_time;
    uint8_t temperature;  // deg C
    uint8_t error_code;
} ddsm210_feedback_t;

typedef struct {
    int32_t  mileage_laps; // signed full revolutions
    uint16_t position;     // 0..65535 = 0..360 deg
    uint8_t  error_code;
} ddsm210_odometry_t;

// Query current operating mode. Returns -1 on timeout, else mode value.
int ddsm210_get_mode(ddsm_port_id_t port, uint8_t motor_id);

// Set operating mode. Returns true if the motor acknowledged.
bool ddsm210_set_mode(ddsm_port_id_t port, uint8_t motor_id, uint8_t mode);

// Velocity-loop drive. rpm_x10 in 0.1 RPM units (clamped to +/-2100),
// accel_time in 0.1 ms-per-RPM units. fb may be NULL.
bool ddsm210_set_velocity(ddsm_port_id_t port, uint8_t motor_id, int16_t rpm_x10,
                          uint8_t accel_time, ddsm210_feedback_t *fb);

// Velocity-loop brake.
bool ddsm210_brake(ddsm_port_id_t port, uint8_t motor_id);

// Position-loop drive. position 0..32767 = 0..360 deg.
bool ddsm210_set_position(ddsm_port_id_t port, uint8_t motor_id, uint16_t position,
                          ddsm210_feedback_t *fb);

// Mileage + absolute encoder readback (command 0x74).
bool ddsm210_get_odometry(ddsm_port_id_t port, uint8_t motor_id, ddsm210_odometry_t *odom);

#endif // MOTORS_DDSM210_H
