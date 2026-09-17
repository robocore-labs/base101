/*
 * The four drive wheels (Waveshare DDSM210, velocity mode).
 *
 * One motor per port: a DDSM210 cannot share a TX line without external
 * gating, so each wheel gets its own PIO UART. Which pins, which corner and
 * which way it spins is the WHEELS table in robot.h; everything here is
 * indexed by position in that table.
 *
 * Wheels that don't answer at boot are remembered as offline and skipped,
 * so a disconnected motor costs one probe at startup and nothing after.
 *
 * wheels_set_speed() sets what a wheel should be doing, not what it does
 * this instant: wheels_update() ramps the real commanded speed toward that
 * target a little at a time (WHEEL_ACCEL_LIMIT_RAD_S2 in robot.h), so a
 * base_cmd that steps -- reversing direction, or a turn asking for very
 * different left/right speeds -- doesn't skid the wheel across the floor
 * chasing an instant jump. Call wheels_update() every time round the main
 * loop; it decides on its own how often that's actually worth acting on.
 */

#ifndef WHEELS_H
#define WHEELS_H

#include <stdbool.h>
#include <stdint.h>

// Open a port per wheel, find out who is there, and put them in velocity
// mode. Returns how many answered.
uint8_t wheels_begin(void);

bool wheels_online(uint8_t index);

// Set the target speed for one wheel, in rad/s at the wheel. Direction and
// the speed cap from robot.h are applied when it's actually sent, not
// here -- this only records what wheels_update() should be steering
// toward.
void wheels_set_speed(uint8_t index, double rad_per_sec);

// Advance the ramp for every wheel toward its target and send whichever
// ones moved. Call every time round the main loop -- it rate-limits
// itself to WHEEL_CONTROL_HZ, so calling it more often than that costs
// nothing.
void wheels_update(void);

// Stop one wheel right now: sets both its target and its ramped speed to
// zero and sends the DDSM210's brake command immediately, skipping the
// ramp entirely. This is the safety path, not the comfortable one --
// used when a wheel must stop with no further argument, not when it
// should ease down to a stop (that's just wheels_set_speed(index, 0),
// same as any other target).
void wheels_brake(uint8_t index);

// Where the wheel is now, in radians, counting full turns since boot.
// Returns 0 for a wheel that is offline or didn't answer this time.
double wheels_read_angle(uint8_t index);

#endif // WHEELS_H
