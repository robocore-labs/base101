/*
 * Axon Pico-ROS node: firmware port of the LLMy ddsm210_manager and
 * st3215_manager ROS 2 packages.
 *
 * Subscribes:
 *   /motor_manager/base_cmd    Float64MultiArray  -> DDSM210 wheels (velocity)
 *   /motor_manager/arm_cmd     Float64MultiArray  -> ST3215 arm (position)
 *   /motor_manager/camera_cmd  Float64MultiArray  -> ST3215 camera (if enabled)
 *
 * Publishes:
 *   /motor_manager/joint_states            JointState (merged, fixed slots)
 *   /motor_telemetry/<joint>/current       Float32 (mA)
 *   /motor_telemetry/<joint>/voltage       Float32 (V)
 *   /motor_telemetry/<joint>/load          Float32 (%)
 *   /motor_telemetry/<joint>/temperature   Int32 (deg C)
 *   /imu/data                              Imu (BNO055 fusion)
 *   /imu/mag                               MagneticField (BNO055)
 *   /imu/temperature                       Temperature (BNO055)
 */

#ifndef AXON_NODE_H
#define AXON_NODE_H

#include <stdbool.h>
#include <stdint.h>

// Initialize motors (modes, torque, hold positions). Call before the
// zenoh session is opened; pure bus I/O.
void axon_node_motors_init(void);

// Initialize the I2C bus and the BNO055 IMU. Call before the zenoh session
// is opened; pure bus I/O. No-op when AXON_IMU_ENABLE is 0.
void axon_imu_init(void);

// Declare node, publishers and subscribers. Call once after
// picoros_interface_init() succeeded.
bool axon_node_declare(void);

// Periodic work: telemetry publishing at the configured rates.
// Call from the main loop (after each picoros_single_threaded_loop()).
void axon_node_spin(void);

// Liveness counters for the debug-port heartbeat (any pointer may be NULL):
// commands received on base_cmd/arm_cmd and joint_states/IMU messages published.
void axon_node_status(uint32_t *base_cmds, uint32_t *arm_cmds,
                      uint32_t *joint_pubs, uint32_t *imu_pubs);

#endif // AXON_NODE_H
