#include "led.h"

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"

#include "pins.h"
#include "ws2812.pio.h"

// PIO0 carries the servo half-duplex bus on SM0/SM1 (14 instructions). The
// WS2812 program is 4 more instructions and takes the spare SM2 — comfortably
// inside PIO0's 32-word store and 4 state machines.
#define LED_PIO       pio0
#define LED_SM        2u
#define LED_FREQ_HZ   800000u

// Breathing profile per mode. Brightness is a raw 8-bit channel value kept
// deliberately dim (WS2812 at 3V3, no per-pixel decoupling on this board).
//   period_ms = one full fade down-and-up
//   lo/hi     = channel value at the trough / peak of the breath
typedef struct {
    uint16_t period_ms;
    uint8_t  lo;
    uint8_t  hi;
} breath_t;

static const breath_t BREATH_CONFIG = { .period_ms = 900,  .lo = 0, .hi = 90 };  // brisk, urgent
static const breath_t BREATH_NORMAL = { .period_ms = 3000, .lo = 3, .hi = 40 };  // slow, calm

static bool       s_ready;
static led_mode_t s_mode;
static uint32_t   s_last_show_us;
static int        s_last_level = -1;

static inline void put_pixel_grb(uint8_t r, uint8_t g, uint8_t b) {
    // WS2812 wants GRB, MSB-first; the PIO program autopulls 24 bits from the
    // top of the word, so the colour sits in bits 31..8.
    uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;
    pio_sm_put_blocking(LED_PIO, LED_SM, grb << 8u);
}

static void fill(uint8_t r, uint8_t g, uint8_t b) {
    for (uint i = 0; i < NEOPIXEL_COUNT; i++) put_pixel_grb(r, g, b);
}

void led_init(led_mode_t mode) {
    if (s_ready) return;
    s_mode = mode;

    pio_sm_claim(LED_PIO, LED_SM);
    uint offset = pio_add_program(LED_PIO, &ws2812_program);
    ws2812_program_init(LED_PIO, LED_SM, offset, PIN_NEOPIXELS, (float)LED_FREQ_HZ, false);
    gpio_set_pulls(PIN_NEOPIXELS, true, false);  // weak pull-up helps signal integrity at 3V3

    s_ready = true;
    s_last_show_us = time_us_32();
    fill(0, 0, 0);  // clear any power-on garbage before the first breath step
}

void led_task(void) {
    if (!s_ready) return;

    uint32_t now = time_us_32();
    if ((now - s_last_show_us) < 12000u) return;  // cap refresh at ~80 Hz
    s_last_show_us = now;

    const breath_t *b = (s_mode == LED_MODE_CONFIG) ? &BREATH_CONFIG : &BREATH_NORMAL;
    uint32_t half = b->period_ms / 2u;
    uint32_t ph   = (now / 1000u) % b->period_ms;          // 0 .. period
    uint32_t tri  = (ph < half) ? ph : (b->period_ms - ph); // triangle, 0 .. half
    uint32_t eased = (tri * tri) / half;                    // ease in/out, 0 .. half
    int level = (int)b->lo + (int)((uint32_t)(b->hi - b->lo) * eased / half);

    if (level == s_last_level) return;  // no visible change: skip the bus traffic
    s_last_level = level;

    if (s_mode == LED_MODE_CONFIG) fill((uint8_t)level, 0, 0);
    else                           fill(0, 0, (uint8_t)level);
}
