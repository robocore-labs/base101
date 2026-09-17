/*
 * PIO UART loopback tester (throwaway rev2 bring-up tool).
 *
 * Validates the real PIO uart_tx -> uart_rx path on a pin pair (jumper TX->RX):
 * sends an 8N1 byte pattern out the TX SM and checks the RX SM receives it.
 *
 * USB serial REPL (drive from host):
 *   "all"          test the 4 new UART pairs: 19/20 21/22 23/24 25/26
 *   "u <tx> <rx>"  (or "<tx> <rx>") test one pair
 * RP2350A: all pins are GP0..29, default PIO window, no gpio_base needed.
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "uart_tx.pio.h"
#include "uart_rx.pio.h"

#define BAUD 115200
static PIO pio = pio0;
static uint tx_off, rx_off;

static void uart_loop(int tx, int rx) {
    pio_uart_tx_program_init(pio, 0, tx_off, (uint)tx, BAUD);
    pio_uart_rx_program_init(pio, 1, rx_off, (uint)rx, BAUD);
    sleep_ms(3);                                  // let TX idle high settle
    while (!pio_sm_is_rx_fifo_empty(pio, 1)) (void)pio_sm_get(pio, 1);

    static const uint8_t pat[] = {0x55, 0xAA, 0x00, 0xFF, 0x3C, 0x81};
    const int N = (int)sizeof(pat);
    for (int i = 0; i < N; i++) pio_sm_put_blocking(pio, 0, pat[i]);

    uint8_t got[16];
    int n = 0;
    absolute_time_t dl = make_timeout_time_ms(50);
    while (n < N && !time_reached(dl)) {
        if (!pio_sm_is_rx_fifo_empty(pio, 1))
            got[n++] = (uint8_t)(pio_sm_get(pio, 1) >> 24);
    }

    int ok = (n == N);
    for (int i = 0; i < n; i++) if (got[i] != pat[i]) ok = 0;

    printf("UART GP%d(TX)->GP%d(RX): %d/%d [", tx, rx, n, N);
    for (int i = 0; i < n; i++) printf("%02X ", got[i]);
    printf("] %s\n", ok ? "OK" : (n ? "GARBLED" : "SILENT"));

    pio_sm_set_enabled(pio, 0, false);
    pio_sm_set_enabled(pio, 1, false);
}

int main(void) {
    stdio_init_all();
    sleep_ms(1500);
    tx_off = pio_add_program(pio, &pio_uart_tx_program);
    rx_off = pio_add_program(pio, &pio_uart_rx_program);

    printf("\n== PIO UART loopback test (115200 8N1) ==\n"
           "  'all' = 19/20 21/22 23/24 25/26 | 'u <tx> <rx>' = one pair\n> ");

    char line[64];
    int n = 0;
    while (true) {
        int c = getchar_timeout_us(1000);
        if (c == PICO_ERROR_TIMEOUT) continue;
        if (c == '\r' || c == '\n') {
            line[n] = 0;
            if (n > 0) {
                int a, b;
                if (line[0] == 'a') {
                    static const int pairs[4][2] = {{19,20},{21,22},{23,24},{25,26}};
                    for (int i = 0; i < 4; i++) uart_loop(pairs[i][0], pairs[i][1]);
                } else if (sscanf(line, "u %d %d", &a, &b) == 2 ||
                           sscanf(line, "%d %d", &a, &b) == 2) {
                    uart_loop(a, b);
                }
            }
            n = 0;
            printf("> ");
        } else if (n < (int)sizeof(line) - 1) {
            line[n++] = (char)c;
            putchar(c);
        }
    }
}
