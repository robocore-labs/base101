/*
 * USB, and the one job that has to keep running no matter what.
 *
 * io_poll() is the firmware's heartbeat. It services USB, moves lidar bytes
 * and breathes the LED, and it runs in two places: from the main loop, and
 * from inside every blocking wait in the firmware -- a servo read, a wheel
 * read, zenoh waiting on a router that isn't up yet. That last one can last
 * minutes, which is exactly when a board needs to still look alive.
 *
 * That second part is what serial_hook_init() is for. Every bus is wrapped
 * so its task() calls io_poll() first, and the drivers call task() while
 * they wait for a reply. So a 2 ms servo transaction is 2 ms of USB and
 * lidar still running, and nothing in the firmware has to remember to keep
 * anything else alive.
 *
 * The one rule: io_poll() must never touch a wrapped bus or call into
 * zenoh. It would be calling itself.
 */

#ifndef IO_H
#define IO_H

// Bring USB up and give a host a moment to attach, so it sees the boot
// narration on the debug port.
void io_begin(void);

// USB, lidar, LED. Call it often; pass it to every bus as the hook.
void io_poll(void);

#endif // IO_H
