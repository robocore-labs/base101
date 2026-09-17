/*
 * The lidar passthrough: USB CDC #1 <-> uart1.
 *
 * The lidar is the one device the host still talks to directly. Scans are
 * high rate and the firmware has no use for them, so bytes are copied
 * between the port and the UART untouched and the driver runs on the host
 * as if the lidar were plugged into it.
 *
 * Both sides are serial_t, so the bridge is a copy loop in both directions
 * and nothing more.
 */

#ifndef LIDAR_H
#define LIDAR_H

#include <stdbool.h>
#include <stdint.h>

// Open uart1 on the lidar pins and the CDC port in front of it.
bool lidar_begin(void);

// Move whatever is waiting, in both directions. Called from io_poll(), so
// it keeps running even while something else blocks on a bus.
void lidar_update(void);

// Follow a baud rate the host asked for on the CDC port, so changing it
// there changes the real UART. Wired up in usb_descriptors.c's line-coding
// callback.
void lidar_set_baudrate(uint32_t baud);

#endif // LIDAR_H
