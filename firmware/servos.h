/*
 * The arm (Feetech STS/SCS servos, position mode).
 *
 * All of them share one half-duplex bus on the board's fixed servo pins,
 * told apart by ID. Which IDs, which joint names and which way each one
 * turns is the SERVOS table in robot.h; everything here is indexed by
 * position in that table.
 *
 * Set SERVOS_ENABLED to false and every function here becomes a no-op --
 * for a robot with no arm on it, so the firmware doesn't spend main-loop
 * time talking to servos that aren't there.
 */

#ifndef SERVOS_H
#define SERVOS_H

#include <stdbool.h>
#include <stdint.h>

#include "st3215.h"

// Open the bus, ping each servo, and put the ones that answer into position
// mode holding where they already are. Returns how many answered.
uint8_t servos_begin(void);

bool servos_online(uint8_t index);

// Move one servo to an angle in radians, measured from centre. Direction
// and the speed/acceleration from robot.h are applied here, and an
// unchanged angle is not re-sent.
void servos_set_angle(uint8_t index, double radians);

// Where the servo is and how fast it is turning, in radians and rad/s.
// Returns false if it didn't answer; the outputs are untouched then.
bool servos_read_state(uint8_t index, double *radians, double *rad_per_sec);

// Current, voltage, load and temperature, straight from the servo.
bool servos_read_telemetry(uint8_t index, st3215_telemetry_t *out);

#endif // SERVOS_H
