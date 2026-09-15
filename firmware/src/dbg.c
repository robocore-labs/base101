#include "dbg.h"

#include <stdarg.h>
#include <stdio.h>

#include "tusb.h"
#include "usb_descriptors.h"

static volatile bool s_usb_enabled = true;

void dbg_enable_usb(bool enabled) {
    s_usb_enabled = enabled;
}

void dbg_printf(const char *fmt, ...) {
    // Silenced once the zenoh session is up (see dbg_enable_usb) so debug output
    // never competes with the zenoh transport for USB bandwidth in steady state.
    if (!s_usb_enabled) return;

    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n >= (int)sizeof(buf)) n = sizeof(buf) - 1;

    // Deliver to the debug CDC whenever the USB device is mounted — NOT gated
    // on DTR (tud_cdc_n_connected). A plain reader like `cat`/`screen` that
    // doesn't assert DTR would otherwise see nothing. tud_cdc_n_write is
    // non-blocking and drops bytes when the FIFO is full, so an unread port can
    // never block or back up the firmware.
    if (tud_mounted()) {
        tud_cdc_n_write(CDC_IDX_DEBUG, buf, (uint32_t)n);
        tud_cdc_n_write_flush(CDC_IDX_DEBUG);
    }
}
