#ifndef AXON_LED_H
#define AXON_LED_H

// WS2812 NeoPixel strip (NEOPIXEL_COUNT LEDs on PIN_NEOPIXELS, see pins.h) used
// as a single at-a-glance mode indicator:
//
//   LED_MODE_CONFIG  -> all pixels breathe RED   (config / bench-test mode)
//   LED_MODE_NORMAL  -> all pixels breathe BLUE  (normal ROS node operation)
//
// Single-core, cooperative: the strip lives on the spare PIO0 state machine and
// is refreshed from led_task(), which must be pumped often from the main loop
// (and from io_poll() so it keeps animating while zenoh blocks). A frozen strip
// therefore means frozen firmware.

typedef enum {
    LED_MODE_CONFIG = 0,
    LED_MODE_NORMAL,
} led_mode_t;

// Claim the PIO state machine and latch the mode. Safe to call once; later
// calls are ignored. Call after half_duplex_init()/ddsm_port_init() so the
// servo-bus UART programs get their PIO0 slots first.
void led_init(led_mode_t mode);

// Advance the breathing animation. Cheap (a few integer ops); only touches the
// PIO FIFO when the brightness step actually changes, at most ~80x/second.
void led_task(void);

#endif // AXON_LED_H
