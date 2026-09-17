#include "status.h"

#include <stdarg.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "tusb.h"

#include "link101/neopixel.h"

#include "robot.h"
#include "usb_descriptors.h"

static bool s_printing = true;
static link101_rgb_t s_pixels[LINK101_NEOPIXEL_COUNT];
static bool s_leds_ready;
static status_mode_t s_mode = STATUS_WAITING;

void status_begin(void) {
#if LED_ENABLED
    s_leds_ready = link101_neopixel_init(LINK101_PIN_NEOPIXELS, s_pixels,
                                         LINK101_NEOPIXEL_COUNT,
                                         LINK101_NEOPIXEL_FREQ);
    if (s_leds_ready) {
        link101_neopixel_clear();   // clear whatever the power-on left behind
        link101_neopixel_show();
    }
#endif
}

void status_set_mode(status_mode_t mode) {
    s_mode = mode;
}

void status_quiet(void) {
    s_printing = false;
}

void status_printf(const char *fmt, ...) {
    if (!s_printing) {
        return;
    }

    char line[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    if (len < 0) {
        return;
    }
    if (len >= (int)sizeof(line)) {
        len = sizeof(line) - 1;
    }

    // Written whenever the device is mounted, NOT when the port is "open":
    // a plain reader like cat or screen never asserts DTR, and would
    // otherwise see nothing. The write is non-blocking and drops bytes when
    // the FIFO fills, so an unread port costs nothing.
    if (tud_mounted()) {
        tud_cdc_n_write(CDC_IDX_DEBUG, line, (uint32_t)len);
        tud_cdc_n_write_flush(CDC_IDX_DEBUG);
    }
}

// A breath: brightness runs LED_MIN -> LED_MAX -> LED_MIN over the period,
// eased so it looks like breathing rather than a triangle wave.
static uint8_t breath_level(uint32_t now_ms, uint32_t period_ms) {
    uint32_t half  = period_ms / 2;
    uint32_t phase = now_ms % period_ms;
    uint32_t ramp  = (phase < half) ? phase : (period_ms - phase);   // 0..half
    uint32_t eased = (ramp * ramp) / half;                           // 0..half
    return (uint8_t)(LED_MIN + (LED_MAX - LED_MIN) * eased / half);
}

// What the strip should be showing right now.
static link101_rgb_t current_colour(uint32_t now_ms) {
    if (s_mode == STATUS_WAITING) {
        // Hard on, hard off. A blink reads as waiting on something; a breath
        // reads as everything being fine, which it is not yet.
        bool on = (now_ms % LED_WAITING_PERIOD_MS) < (LED_WAITING_PERIOD_MS / 2);
        return on ? (link101_rgb_t){ .r = LED_WAITING_R, .g = LED_WAITING_G, .b = 0 }
                  : (link101_rgb_t){ .r = 0, .g = 0, .b = 0 };
    }
    return (link101_rgb_t){ .r = 0, .g = breath_level(now_ms, LED_READY_PERIOD_MS), .b = 0 };
}

void status_update(void) {
#if LED_ENABLED
    static uint32_t next_frame_us = 0;
    static link101_rgb_t last = { 0xFF, 0xFF, 0xFF };   // nothing shows this

    if (!s_leds_ready) {
        return;
    }

    uint32_t now = time_us_32();
    if ((int32_t)(now - next_frame_us) < 0) {
        return;
    }
    next_frame_us = now + 12000;   // ~80 frames a second is plenty

    link101_rgb_t colour = current_colour(now / 1000);
    if (colour.r == last.r && colour.g == last.g && colour.b == last.b) {
        return;                    // nothing visible would change
    }
    last = colour;

    link101_neopixel_fill(colour);
    link101_neopixel_show();
#endif
}
