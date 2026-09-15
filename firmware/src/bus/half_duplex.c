#include "half_duplex.h"
#include "pins.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "tusb.h"

#include "uart_tx.pio.h"
#include "uart_rx.pio.h"

// Unified servo bus over a single physical half-duplex channel.
//
// The Axon 2 board exposes one servo connector (Feetech STS/SCS, also speaks
// Dynamixel) wired as a half-duplex UART with an external TX-enable line for
// direction control. It is driven by a PIO UART on pio0 (SM0 TX / SM1 RX).
//
// TXEN quirk (mirrors the upstream "default" firmware): the enable line must
// stay asserted until the LAST byte's stop bit has fully shifted out of the TX
// SM, otherwise the final byte is truncated on the wire. pio_sm_is_tx_fifo_empty
// only reports the FIFO, not the shift register, so we add a ~2 byte-time
// settle delay before releasing the bus back to RX.

#define HD_PIO   pio0
#define HD_SM_TX 0
#define HD_SM_RX 1

#define HD_RX_BUF_SIZE 256
static uint8_t rx_buf[HD_RX_BUF_SIZE];
static volatile uint32_t rx_head = 0;
static volatile uint32_t rx_tail = 0;

static bool initialized = false;
static uint32_t current_baudrate = 0;
static uint tx_offset = 0;
static uint rx_offset = 0;

void half_duplex_init(uint32_t baudrate) {
    gpio_init(PIN_MOTOR_TXEN);
    gpio_set_dir(PIN_MOTOR_TXEN, GPIO_OUT);
    gpio_put(PIN_MOTOR_TXEN, 0);  // default to RX mode

    tx_offset = pio_add_program(HD_PIO, &pio_uart_tx_program);
    rx_offset = pio_add_program(HD_PIO, &pio_uart_rx_program);

    pio_uart_tx_program_init(HD_PIO, HD_SM_TX, tx_offset, PIN_MOTOR_TX, baudrate);
    pio_uart_rx_program_init(HD_PIO, HD_SM_RX, rx_offset, PIN_MOTOR_RX, baudrate);

    current_baudrate = baudrate;
    rx_head = rx_tail = 0;
    initialized = true;
}

void half_duplex_set_baudrate(uint32_t baudrate) {
    if (!initialized || baudrate == current_baudrate) return;

    float div = (float)clock_get_hz(clk_sys) / (baudrate * 8);
    pio_sm_set_enabled(HD_PIO, HD_SM_TX, false);
    pio_sm_set_enabled(HD_PIO, HD_SM_RX, false);
    pio_sm_clear_fifos(HD_PIO, HD_SM_TX);
    pio_sm_clear_fifos(HD_PIO, HD_SM_RX);
    pio_sm_set_clkdiv(HD_PIO, HD_SM_TX, div);
    pio_sm_set_clkdiv(HD_PIO, HD_SM_RX, div);
    pio_sm_set_enabled(HD_PIO, HD_SM_TX, true);
    pio_sm_set_enabled(HD_PIO, HD_SM_RX, true);
    current_baudrate = baudrate;
}

uint32_t half_duplex_write(const uint8_t *data, uint32_t len) {
    if (!initialized || len == 0) return 0;

    gpio_put(PIN_MOTOR_TXEN, 1);  // drive the bus

    for (uint32_t j = 0; j < len; j++) {
        pio_sm_put_blocking(HD_PIO, HD_SM_TX, (uint32_t)data[j]);
    }

    // Wait for the TX FIFO to drain, then hold TXEN through the final stop bit.
    while (!pio_sm_is_tx_fifo_empty(HD_PIO, HD_SM_TX)) {
        tight_loop_contents();
    }
    uint32_t byte_time_us = (10 * 1000000) / current_baudrate;
    if (byte_time_us < 10) byte_time_us = 10;
    sleep_us(byte_time_us * 2);

    gpio_put(PIN_MOTOR_TXEN, 0);  // release the bus for RX
    return len;
}

uint32_t half_duplex_read(uint8_t *data, uint32_t max_len) {
    if (!initialized) return 0;

    uint32_t count = 0;
    while (count < max_len && rx_head != rx_tail) {
        data[count++] = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1) % HD_RX_BUF_SIZE;
    }
    return count;
}

uint32_t half_duplex_available(void) {
    if (!initialized) return 0;
    return (rx_head - rx_tail + HD_RX_BUF_SIZE) % HD_RX_BUF_SIZE;
}

void half_duplex_task(void) {
    if (!initialized) return;

    while (!pio_sm_is_rx_fifo_empty(HD_PIO, HD_SM_RX)) {
        uint8_t c = (uint8_t)(pio_sm_get(HD_PIO, HD_SM_RX) >> 24);
        uint32_t next_head = (rx_head + 1) % HD_RX_BUF_SIZE;
        if (next_head != rx_tail) {
            rx_buf[rx_head] = c;
            rx_head = next_head;
        }
    }
}

int half_duplex_transact(const uint8_t *tx_data, uint32_t tx_len,
                         uint8_t *rx_data, uint32_t rx_max_len,
                         uint32_t timeout_us) {
    if (!initialized) return -1;

    rx_head = rx_tail = 0;
    pio_sm_clear_fifos(HD_PIO, HD_SM_RX);

    half_duplex_write(tx_data, tx_len);

    absolute_time_t deadline = make_timeout_time_us(timeout_us);
    uint32_t rx_count = 0;

    while (rx_count < rx_max_len) {
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) break;

        while (!pio_sm_is_rx_fifo_empty(HD_PIO, HD_SM_RX) && rx_count < rx_max_len) {
            rx_data[rx_count++] = (uint8_t)(pio_sm_get(HD_PIO, HD_SM_RX) >> 24);
        }
        // Keep USB serviced while we busy-wait on the servo bus, so probing
        // offline servos (each blocking up to timeout_us) can't starve
        // tud_task() long enough for the host to drop the device. Mirrors the
        // DDSM driver's poll. Safe in both config and normal mode.
        tud_task();
    }
    return (int)rx_count;
}
