/*
 * Debug logging to USB CDC #2.
 *
 * For low-rate, human-readable status: initialization banners, which motor
 * IDs were found, bus/zenoh state. NOT for per-message traffic. Output goes
 * to the debug CDC port while enabled (see dbg_enable_usb).
 *
 * It is silenced once the zenoh session is up so the debug stream never
 * competes with the zenoh transport for USB bandwidth in steady state.
 */

#ifndef AXON_DBG_H
#define AXON_DBG_H

#include <stdbool.h>

void dbg_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// Enable/disable debug output to the USB CDC. Enabled at boot; main() turns it
// off after the zenoh session connects. When disabled, dbg_printf() is a no-op.
void dbg_enable_usb(bool enabled);

#endif // AXON_DBG_H
