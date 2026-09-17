#include "lidar_uart.h"

#include "pins.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

// Lidar on the uart1 hardware peripheral (GPIO4 TX / GPIO5 RX), forwarded
// over USB CDC #1. Incoming scan data is drained by an RX interrupt into a
// ring buffer so a slow main-loop poll cannot overflow the 32-byte hardware
// FIFO at the lidar's baud rate.
#define LIDAR_UART  uart1
#define LIDAR_IRQ   UART1_IRQ

#define LIDAR_RX_BUF_SIZE 2048

static uint8_t rx_buf[LIDAR_RX_BUF_SIZE];
static volatile uint32_t rx_head = 0;
static volatile uint32_t rx_tail = 0;

static bool initialized = false;
static uint32_t current_baudrate = 0;

static void lidar_uart_rx_irq(void) {
    while (uart_is_readable(LIDAR_UART)) {
        uint8_t c = (uint8_t)uart_getc(LIDAR_UART);
        uint32_t next_head = (rx_head + 1) % LIDAR_RX_BUF_SIZE;
        if (next_head != rx_tail) {
            rx_buf[rx_head] = c;
            rx_head = next_head;
        }
    }
}

void lidar_uart_init(uint32_t baudrate) {
    uart_init(LIDAR_UART, baudrate);
    gpio_set_function(PIN_LIDAR_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_LIDAR_RX, GPIO_FUNC_UART);
    uart_set_hw_flow(LIDAR_UART, false, false);
    uart_set_format(LIDAR_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(LIDAR_UART, true);

    irq_set_exclusive_handler(LIDAR_IRQ, lidar_uart_rx_irq);
    irq_set_enabled(LIDAR_IRQ, true);
    uart_set_irq_enables(LIDAR_UART, true, false);  // RX only

    current_baudrate = baudrate;
    rx_head = rx_tail = 0;
    initialized = true;
}

void lidar_uart_set_baudrate(uint32_t baudrate) {
    if (!initialized || baudrate == 0 || baudrate == current_baudrate) return;
    uart_set_baudrate(LIDAR_UART, baudrate);
    current_baudrate = baudrate;
}

uint32_t lidar_uart_write(const uint8_t *data, uint32_t len) {
    if (!initialized || len == 0) return 0;
    uart_write_blocking(LIDAR_UART, data, len);
    return len;
}

uint32_t lidar_uart_read(uint8_t *data, uint32_t max_len) {
    if (!initialized) return 0;

    uint32_t count = 0;
    while (count < max_len && rx_head != rx_tail) {
        data[count++] = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1) % LIDAR_RX_BUF_SIZE;
    }
    return count;
}

uint32_t lidar_uart_available(void) {
    if (!initialized) return 0;
    return (rx_head - rx_tail + LIDAR_RX_BUF_SIZE) % LIDAR_RX_BUF_SIZE;
}

// RX is serviced by the interrupt handler; nothing to poll here. Kept so the
// main loop's call site is unchanged.
void lidar_uart_task(void) {
}
