/*
 * Runtime configuration, persisted to internal flash.
 *
 * Holds the parts of the robot setup the user can change without reflashing:
 * whether the ST3215 servo subsystem is active at all, and the list of servos
 * (count + ID -> joint-name mapping). The DDSM wheel motors are fixed in
 * firmware (axon_config.h); only the servos are runtime-configurable.
 *
 * Changes are edited in RAM (e.g. over the debug-port JSON console), saved to
 * flash, and applied on the next boot — the motor tables, ROS publishers and
 * joint_states layout are all built once at startup from this config.
 */

#ifndef AXON_CFG_H
#define AXON_CFG_H

#include <stdbool.h>
#include <stdint.h>

#define AXON_SERVO_MAX      16
#define AXON_JOINT_NAME_LEN 16

typedef struct {
    char    joint[AXON_JOINT_NAME_LEN];  // joint name / telemetry topic token
    uint8_t id;                          // ST3215 bus ID
    bool    enable;                      // include this servo
} axon_servo_cfg_t;

typedef struct {
    bool             servos_enabled;     // master gate for the ST3215 subsystem
    uint8_t          servo_count;        // valid entries in servos[]
    axon_servo_cfg_t servos[AXON_SERVO_MAX];
} axon_cfg_t;

// The single in-RAM configuration instance (loaded at boot, edited by the
// console, saved to flash). Treat as read-mostly outside the console.
extern axon_cfg_t g_cfg;

// Populate g_cfg with compiled-in defaults (servos disabled, the stock arm
// servo list). Does not touch flash.
void axon_cfg_defaults(void);

// Load g_cfg from flash; falls back to defaults if no valid config is stored.
// Returns true if a valid stored config was loaded.
bool axon_cfg_load(void);

// Persist the current g_cfg to flash. Returns true on success. Briefly
// disables interrupts (and USB) while the flash sector is erased/written.
bool axon_cfg_save(void);

#endif // AXON_CFG_H
