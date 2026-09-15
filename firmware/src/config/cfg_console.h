/*
 * Interactive JSON configuration console on the debug USB CDC (CDC #2).
 *
 * One JSON object per line (newline-terminated). Commands:
 *   {"cmd":"get"}        -> prints the current config as JSON
 *   {"cmd":"defaults"}   -> reset config in RAM to compiled defaults
 *   {"cmd":"set", ...}   -> update fields in RAM (see below)
 *   {"cmd":"save"}       -> persist the RAM config to flash
 *   {"cmd":"reboot"}     -> restart (applies a saved config)
 *
 * "set" fields (any subset):
 *   "servos_enabled": true|false
 *   "servos": [ {"id":1,"joint":"shoulder","enable":true}, ... ]
 *
 * Edits apply on the next boot (the motor tables / publishers are built at
 * startup), so the usual flow is set -> save -> reboot.
 */

#ifndef AXON_CFG_CONSOLE_H
#define AXON_CFG_CONSOLE_H

// Poll the debug CDC for command lines and handle them. Call from the main
// I/O poll loop.
void cfg_console_task(void);

#endif // AXON_CFG_CONSOLE_H
