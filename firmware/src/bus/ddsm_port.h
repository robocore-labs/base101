/*
 * Dedicated serial ports for the DDSM210 wheel motors.
 *
 * There are four motors (two per side). Each DDSM210 sits alone on its own
 * full-duplex UART (the motor cannot share a TX line without external
 * gating), implemented with PIO state machines. Pin assignments and the
 * PIO/SM allocation live in pins.h.
 */

#ifndef BUS_DDSM_PORT_H
#define BUS_DDSM_PORT_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    DDSM_FR = 0,   // front right
    DDSM_FL,       // front left
    DDSM_BR,       // back right
    DDSM_BL,       // back left
    DDSM_PORT_COUNT
} ddsm_port_id_t;

void ddsm_port_init(uint32_t baudrate);

uint32_t ddsm_port_write(ddsm_port_id_t id, const uint8_t *data, uint32_t len);
uint32_t ddsm_port_read(ddsm_port_id_t id, uint8_t *data, uint32_t max_len);
uint32_t ddsm_port_available(ddsm_port_id_t id);

// Drain the PIO RX FIFOs into the ring buffers — call often.
void ddsm_port_task(void);

#endif // BUS_DDSM_PORT_H
