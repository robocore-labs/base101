/*
 * Telling the human what is going on: the debug port and the LED strip.
 *
 * status_printf() narrates the boot over USB CDC #2 -- what was found, what
 * answered, what didn't. The LED strip says the same thing from across the
 * room, in the only two states that matter once the boot log is gone:
 *
 *   STATUS_WAITING   fast yellow blink -- no ROS router yet
 *   STATUS_READY     slow green breath -- connected and running
 *
 * Once zenoh is up the debug port goes quiet (status_quiet()), because in
 * steady state every byte of USB bandwidth belongs to the transport. The
 * LED carries on, so the board still tells you it is alive -- and if it
 * ever stops moving, the main loop stopped turning.
 */

#ifndef STATUS_H
#define STATUS_H

#include <stdbool.h>

typedef enum {
    STATUS_WAITING = 0,   // waiting for the ROS router
    STATUS_READY,         // connected and running
} status_mode_t;

// Claim the LED strip, starting in STATUS_WAITING. USB is already up by
// this point; see io_begin().
void status_begin(void);

// Switch what the strip is saying. Takes effect on the next
// status_update().
void status_set_mode(status_mode_t mode);

// Print a line to the debug port. For boot narration and state changes --
// not for per-message traffic. Bytes are dropped rather than blocking when
// nobody is reading, so an unopened port can never stall the firmware.
void status_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// Stop printing. Called once the session is up; there is no way back, and
// that is deliberate.
void status_quiet(void);

// Advance the breathing animation. Cheap, and only touches the LEDs when
// the brightness actually changes. Call it often.
void status_update(void);

#endif // STATUS_H
