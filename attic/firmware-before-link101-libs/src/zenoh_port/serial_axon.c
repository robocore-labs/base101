/*
 * zenoh-pico serial link over the Axon USB CDC port.
 *
 * The zenoh serial protocol (COBS framing + CRC32, serial_protocol.c)
 * sits on top of this byte transport. The framing layer reads one byte
 * at a time until the 0x00 COBS delimiter, so this layer can implement
 * cooperative blocking:
 *
 *   - at a frame boundary (last delivered byte was the 0x00 delimiter,
 *     or nothing delivered yet) a short timeout applies; returning 0
 *     makes the framing layer report SIZE_MAX, which the datagram
 *     transport treats as "no data" — the single-threaded main loop
 *     keeps running.
 *   - mid-frame a longer timeout applies so a frame in flight is not
 *     corrupted by an early bailout.
 *
 * While waiting, the registered idle poll keeps TinyUSB, the lidar
 * bridge and the motor drivers serviced.
 *
 * Locator used by the firmware: "serial/cdc#baudrate=921600"
 * (the baudrate is parsed by zenoh but meaningless on USB CDC).
 */

#include <string.h>

#include "pico/time.h"
#include "tusb.h"

#include "zenoh-pico/config.h"
#include "zenoh-pico/link/transport/serial.h"
#include "zenoh-pico/utils/result.h"

#include "axon_zenoh.h"

#if Z_FEATURE_LINK_SERIAL == 1

extern void axon_zenoh_idle_poll(void);

// True when the next byte handed to the framing layer starts a new
// COBS frame.
static bool s_at_frame_boundary = true;

// Adjustable first-byte timeout: long during session setup (the INIT/ACK
// handshake response can take tens of ms), short during normal operation
// so zp_read never stalls the main loop.
static uint32_t s_poll_timeout_ms = AXON_ZENOH_POLL_TIMEOUT_MS;

void axon_zenoh_set_poll_timeout_ms(uint32_t ms) { s_poll_timeout_ms = ms; }

// After (re)opening the link, zenoh runs its INIT/ACK handshake, whose
// replies take far longer than the steady-state poll timeout. Give every
// fresh link a grace window with a generous first-byte timeout so
// auto-reconnect works without the application's involvement.
#define HANDSHAKE_GRACE_MS 5000
#define HANDSHAKE_POLL_TIMEOUT_MS 500
static absolute_time_t s_grace_until;

static uint32_t boundary_timeout_ms(void) {
    if (!time_reached(s_grace_until) && s_poll_timeout_ms < HANDSHAKE_POLL_TIMEOUT_MS) {
        return HANDSHAKE_POLL_TIMEOUT_MS;
    }
    return s_poll_timeout_ms;
}

z_result_t _z_serial_open_from_dev(_z_sys_net_socket_t *sock, const char *dev, uint32_t baudrate) {
    (void)baudrate;
    if (strcmp(dev, "cdc") != 0) {
        return _Z_ERR_INVALID;
    }
    s_at_frame_boundary = true;
    s_grace_until = make_timeout_time_ms(HANDSHAKE_GRACE_MS);
    sock->_open = true;
    return _Z_RES_OK;
}

z_result_t _z_serial_open_from_pins(_z_sys_net_socket_t *sock, uint32_t txpin, uint32_t rxpin, uint32_t baudrate) {
    (void)sock;
    (void)txpin;
    (void)rxpin;
    (void)baudrate;
    return _Z_ERR_GENERIC;
}

z_result_t _z_serial_listen_from_dev(_z_sys_net_socket_t *sock, const char *dev, uint32_t baudrate) {
    (void)sock;
    (void)dev;
    (void)baudrate;
    return _Z_ERR_GENERIC;
}

z_result_t _z_serial_listen_from_pins(_z_sys_net_socket_t *sock, uint32_t txpin, uint32_t rxpin, uint32_t baudrate) {
    (void)sock;
    (void)txpin;
    (void)rxpin;
    (void)baudrate;
    return _Z_ERR_GENERIC;
}

void _z_serial_close(_z_sys_net_socket_t *sock) { sock->_open = false; }

size_t _z_serial_read(_z_sys_net_socket_t sock, uint8_t *ptr, size_t len) {
    if (!sock._open || len == 0) {
        return 0;
    }

    uint32_t timeout_ms = s_at_frame_boundary ? boundary_timeout_ms() : AXON_ZENOH_FRAME_TIMEOUT_MS;
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);

    size_t n = 0;
    while (n < len) {
        uint32_t avail = tud_cdc_n_available(AXON_ZENOH_CDC_ITF);
        if (avail > 0) {
            uint32_t want = len - n;
            uint32_t got = tud_cdc_n_read(AXON_ZENOH_CDC_ITF, &ptr[n], (want < avail) ? want : avail);
            if (got > 0) {
                n += got;
                // Once a byte arrived, allow the full frame timeout for the rest.
                if (s_at_frame_boundary) {
                    s_at_frame_boundary = false;
                    deadline = make_timeout_time_ms(AXON_ZENOH_FRAME_TIMEOUT_MS);
                }
                continue;
            }
        }
        if (time_reached(deadline)) {
            break;
        }
        axon_zenoh_idle_poll();
    }

    if (n > 0 && ptr[n - 1] == 0x00) {
        // COBS frame delimiter delivered: next read starts a new frame.
        s_at_frame_boundary = true;
    }
    return n;
}

size_t _z_serial_write(_z_sys_net_socket_t sock, const uint8_t *ptr, size_t len) {
    if (!sock._open) {
        return SIZE_MAX;
    }
    if (!tud_cdc_n_connected(AXON_ZENOH_CDC_ITF)) {
        // Host not attached to the zenoh CDC port: drop the frame but keep
        // the link object alive so the session can recover later.
        return SIZE_MAX;
    }

    size_t written = 0;
    absolute_time_t deadline = make_timeout_time_ms(AXON_ZENOH_FRAME_TIMEOUT_MS);
    while (written < len) {
        uint32_t wrote = tud_cdc_n_write(AXON_ZENOH_CDC_ITF, &ptr[written], len - written);
        written += wrote;
        tud_cdc_n_write_flush(AXON_ZENOH_CDC_ITF);
        if (written < len) {
            if (time_reached(deadline)) {
                return SIZE_MAX;
            }
            axon_zenoh_idle_poll();
        }
    }
    tud_cdc_n_write_flush(AXON_ZENOH_CDC_ITF);
    return written;
}

#endif /* Z_FEATURE_LINK_SERIAL == 1 */
