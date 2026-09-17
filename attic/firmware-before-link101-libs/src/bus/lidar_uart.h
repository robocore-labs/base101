/*
 * Lidar UART (PIO) — the one serial port still forwarded over USB.
 * CDC #1 <-> this UART, see main.c.
 */

#ifndef BUS_LIDAR_UART_H
#define BUS_LIDAR_UART_H

#include <stdint.h>

void lidar_uart_init(uint32_t baudrate);
void lidar_uart_set_baudrate(uint32_t baudrate);

uint32_t lidar_uart_write(const uint8_t *data, uint32_t len);
uint32_t lidar_uart_read(uint8_t *data, uint32_t max_len);
uint32_t lidar_uart_available(void);

// Drain the PIO RX FIFO into the ring buffer — call often.
void lidar_uart_task(void);

#endif // BUS_LIDAR_UART_H
