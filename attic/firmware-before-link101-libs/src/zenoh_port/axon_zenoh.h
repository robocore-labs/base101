/*
 * Glue between the zenoh-pico Axon platform port and the rest of the
 * firmware. The platform port must never call zenoh API functions from
 * inside its wait loops, so the firmware registers a background poll
 * callback that keeps USB / motors / LEDs serviced while zenoh blocks
 * on serial I/O.
 */

#ifndef AXON_ZENOH_H
#define AXON_ZENOH_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// CDC port carrying the zenoh serial protocol (set in usb_descriptors.h)
#ifndef AXON_ZENOH_CDC_ITF
#define AXON_ZENOH_CDC_ITF 0
#endif

// Called repeatedly while the zenoh port waits for serial data or sleeps.
// Must NOT call any zenoh/picoros function (no reentrancy).
typedef void (*axon_idle_fn)(void);
void axon_zenoh_set_idle_poll(axon_idle_fn fn);

// First-byte wait used by zp_read before reporting "no data". Set this
// long (e.g. 1000 ms) around session setup so the INIT/ACK handshake can
// complete, then short (AXON_ZENOH_POLL_TIMEOUT_MS) for the main loop.
void axon_zenoh_set_poll_timeout_ms(uint32_t ms);

// Timeout waiting for the first byte of a serial frame before reporting
// "no data" to zenoh (keeps the single-threaded main loop running).
#ifndef AXON_ZENOH_POLL_TIMEOUT_MS
#define AXON_ZENOH_POLL_TIMEOUT_MS 2
#endif

// Timeout for bytes in the middle of a frame (host stalled mid-frame).
#ifndef AXON_ZENOH_FRAME_TIMEOUT_MS
#define AXON_ZENOH_FRAME_TIMEOUT_MS 200
#endif

#ifdef __cplusplus
}
#endif

#endif /* AXON_ZENOH_H */
