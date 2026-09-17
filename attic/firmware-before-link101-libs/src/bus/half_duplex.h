#ifndef BUS_HALF_DUPLEX_H
#define BUS_HALF_DUPLEX_H

#include <stdint.h>
#include <stdbool.h>

// Unified half-duplex motor bus (Feetech STS/SCS, Dynamixel).
// Pins: TX=PIN_MOTOR_TX, RX=PIN_MOTOR_RX (auto-direction hardware on the PCB).
// Uses PIO0 SM0 (TX) and SM1 (RX).

void half_duplex_init(uint32_t baudrate);

void half_duplex_set_baudrate(uint32_t baudrate);

// Send command packet and receive response.
// Returns response length, or -1 on timeout/error.
int half_duplex_transact(const uint8_t *tx_data, uint32_t tx_len,
                         uint8_t *rx_data, uint32_t rx_max_len,
                         uint32_t timeout_us);

uint32_t half_duplex_write(const uint8_t *data, uint32_t len);

uint32_t half_duplex_read(uint8_t *data, uint32_t max_len);

uint32_t half_duplex_available(void);

// Poll task — call from main loop.
void half_duplex_task(void);

#endif // BUS_HALF_DUPLEX_H
