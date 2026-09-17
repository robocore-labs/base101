#include "lidar.h"

#include "cdc_serial.h"
#include "link101/uart_serial.h"

#include "robot.h"
#include "usb_descriptors.h"

// A lidar streams; a stalled main loop must not cost us bytes. RX is
// interrupt-driven into this ring, which at 460800 baud holds about 45 ms
// of scan data.
static uint8_t               uart_buf[2048];
static link101_uart_serial_t uart_port;
static cdc_serial_t          cdc_port;

static serial_t *uart;   // the lidar itself
static serial_t *host;   // the host, through USB CDC #1

bool lidar_begin(void) {
    host = cdc_serial_init(&cdc_port, CDC_IDX_LIDAR);
    uart = link101_uart_serial_init(&uart_port, uart1,
                                    LIDAR_TX_PIN, LIDAR_RX_PIN,
                                    LINK101_NO_DE, LIDAR_BAUD,
                                    uart_buf, sizeof(uart_buf));
    return uart != NULL && host != NULL;
}

void lidar_set_baudrate(uint32_t baud) {
    if (uart) {
        serial_set_baudrate(uart, baud);
    }
}

// Copy everything waiting on `from` to `to`, a chunk at a time.
static void pump(serial_t *from, serial_t *to) {
    uint8_t chunk[64];
    serial_task(from);
    for (;;) {
        uint32_t n = serial_read(from, chunk, sizeof(chunk));
        if (n == 0) {
            return;
        }
        serial_write(to, chunk, n);
    }
}

void lidar_update(void) {
    if (!uart || !host) {
        return;
    }
    pump(host, uart);   // commands down
    pump(uart, host);   // scans up
}
