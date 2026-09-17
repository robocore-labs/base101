/*
 * Minimal GPIO echo/loopback tester (throwaway bring-up tool).
 *
 * USB serial REPL — drive from the host:
 *   "<a> <b>"  -> bidirectional loopback between GPa and GPb (jumper them)
 *   "s <a>"    -> self-readback: drive GPa and read its own pad
 * Pure SIO, no PIO/UART/peripherals. Flashes as axon_firmware.uf2 (temporary).
 */
#include <stdio.h>
#include "pico/stdlib.h"

static void test_pair(int a, int b) {
    gpio_init(a); gpio_set_dir(a, GPIO_OUT);
    gpio_init(b); gpio_set_dir(b, GPIO_IN); gpio_pull_down(b);
    gpio_put(a, 0); sleep_us(300); int a0 = gpio_get(b);
    gpio_put(a, 1); sleep_us(300); int a1 = gpio_get(b);
    gpio_set_dir(a, GPIO_IN); gpio_pull_down(a);
    gpio_set_dir(b, GPIO_OUT);
    gpio_put(b, 0); sleep_us(300); int b0 = gpio_get(a);
    gpio_put(b, 1); sleep_us(300); int b1 = gpio_get(a);
    gpio_deinit(a); gpio_deinit(b);
    printf("GP%d<->GP%d  A->B[%d,%d]  B->A[%d,%d]  %s\n",
           a, b, a0, a1, b0, b1,
           (a0 == 0 && a1 == 1 && b0 == 0 && b1 == 1) ? "LINKED" : "open");
}

static void test_self(int p) {
    gpio_init(p); gpio_set_dir(p, GPIO_OUT);
    gpio_put(p, 0); sleep_us(200); int s0 = gpio_get(p);
    gpio_put(p, 1); sleep_us(200); int s1 = gpio_get(p);
    gpio_deinit(p);
    printf("GP%d self[%d%d]  (want 0,1 = drives & reads)\n", p, s0, s1);
}

int main(void) {
    stdio_init_all();
    sleep_ms(1500);
    printf("\n== GPIO echo test ==\n"
           "  '<a> <b>' = loopback pair (jumper them) | 's <a>' = self-readback\n> ");

    char line[64];
    int n = 0;
    while (true) {
        int c = getchar_timeout_us(1000);
        if (c == PICO_ERROR_TIMEOUT) continue;
        if (c == '\r' || c == '\n') {
            line[n] = 0;
            if (n > 0) {
                int a, b;
                if (line[0] == 's' || line[0] == 'S') {
                    if (sscanf(line + 1, "%d", &a) == 1) test_self(a);
                } else if (sscanf(line, "%d %d", &a, &b) == 2) {
                    test_pair(a, b);
                } else if (sscanf(line, "%d", &a) == 1) {
                    test_self(a);
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
