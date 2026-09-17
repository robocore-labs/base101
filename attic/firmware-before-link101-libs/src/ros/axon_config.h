/*
 * Axon ROS node configuration.
 *
 * This is the firmware equivalent of the LLMy host-side YAML configs:
 *   - ddsm210_manager/config/ddsm210.yaml  (wheel motors)
 *   - st3215_manager/config/llmy.yaml      (arm / camera servos)
 *
 * Edit and reflash to change motor IDs, joint mapping or rates.
 */

#ifndef AXON_CONFIG_H
#define AXON_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/i2c.h"

#include "../bus/ddsm_port.h"

// ---------------------------------------------------------------------------
// Node / transport
// ---------------------------------------------------------------------------
#define AXON_NODE_NAME        "axon"
#define AXON_ROS_DOMAIN_ID    0
#define AXON_ZENOH_MODE       "client"
#define AXON_ZENOH_LOCATOR    "serial/cdc#baudrate=921600"

// ---------------------------------------------------------------------------
// Topics (same as the original packages)
// ---------------------------------------------------------------------------
#define AXON_JOINT_STATES_TOPIC  "motor_manager/joint_states"
#define AXON_BASE_CMD_TOPIC      "motor_manager/base_cmd"
#define AXON_ARM_CMD_TOPIC       "motor_manager/arm_cmd"
#define AXON_CAMERA_CMD_TOPIC    "motor_manager/camera_cmd"
#define AXON_MOTOR_TELEMETRY_PREFIX "motor_telemetry"

// ---------------------------------------------------------------------------
// Rates
// ---------------------------------------------------------------------------
#define AXON_JOINT_STATES_RATE_HZ    50
#define AXON_MOTOR_TELEMETRY_ENABLE  1
// Per tick one servo's current/voltage/load/temperature set is read and
// published (round-robin), keeping bus time per main-loop cycle bounded.
#define AXON_MOTOR_TELEMETRY_RATE_HZ 50

// The JointState message is sized at runtime: the 4 wheels (slots 0..3) plus
// however many servos are enabled in the runtime config (slots 4..). When the
// servo subsystem is disabled it carries just the 4 wheels.

// ---------------------------------------------------------------------------
// DDSM210 wheel motors (velocity mode)
//
// Four motors, two per side, each on its own dedicated serial port (the
// DDSM210 cannot share a TX line without external gating). Pins / PIO map in
// pins.h. The wheels are exposed as four INDEPENDENT joints — base_cmd is a
// 4-element Float64MultiArray of wheel velocities (rad/s) in this order:
//
//     [0] front_left   [1] front_right   [2] back_left   [3] back_right
//
// command_index below must match how ros2_control orders the command array.
//
// motor_id: each motor is alone on its own UART, so the IDs need not be
// unique — every motor can keep the DDSM210 factory-default ID (1). Change a
// row's motor_id only if that specific motor was re-addressed.
// ---------------------------------------------------------------------------
#define AXON_DDSM_COUNT 4
#define AXON_DDSM_BAUD  115200

typedef struct {
    uint8_t         motor_id;
    ddsm_port_id_t  port;
    const char     *joint_name;
    int8_t          direction;      // +1 / -1
    uint8_t         command_index;  // index into base_cmd Float64MultiArray
    uint8_t         state_index;    // slot in JointState arrays
} axon_ddsm_motor_cfg_t;

static const axon_ddsm_motor_cfg_t AXON_DDSM_MOTORS[AXON_DDSM_COUNT] = {
    { .motor_id = 1, .port = DDSM_FL, .joint_name = "front_left_wheel_joint",
      .direction = -1, .command_index = 0, .state_index = 0 },
    { .motor_id = 1, .port = DDSM_FR, .joint_name = "front_right_wheel_joint",
      .direction = 1,  .command_index = 1, .state_index = 1 },
    { .motor_id = 1, .port = DDSM_BL, .joint_name = "back_left_wheel_joint",
      .direction = -1, .command_index = 2, .state_index = 2 },
    { .motor_id = 1, .port = DDSM_BR, .joint_name = "back_right_wheel_joint",
      .direction = 1,  .command_index = 3, .state_index = 3 },
};

#define AXON_DDSM_SPEED_SCALE 1.0
#define AXON_DDSM_ACCEL_TIME  20    // 0.1 ms per RPM units (20 = 2 ms/RPM)
#define AXON_DDSM_MAX_RPM     100   // command cap; hardware max is 210

// ---------------------------------------------------------------------------
// ST3215 servos on the (merged) half-duplex Feetech bus @ 1 Mbaud.
//
// The servo LIST (how many, their IDs and joint names) is runtime config —
// edited over the debug-port JSON console / web UI and stored in flash; see
// src/config/axon_cfg.h. Whether the subsystem runs at all is the runtime
// flag `g_cfg.servos_enabled` (off by default). The servos are driven in
// POSITION mode on AXON_ARM_CMD_TOPIC; these are the shared loop parameters.
// ---------------------------------------------------------------------------
#define AXON_ST_BAUD 1000000
#define AXON_ST_TICKS_PER_REV 4096

#define AXON_SERVO_SPEED_SCALE     0.5
#define AXON_SERVO_POSITION_SPEED  200   // steps/s base for MoveTo
#define AXON_SERVO_POSITION_ACCEL  200
#define AXON_SERVO_DEFAULT_ACCEL   20

// ---------------------------------------------------------------------------
// BNO055 IMU (NDOF fusion mode) on hardware I2C (i2c1, pins in pins.h)
// ---------------------------------------------------------------------------
#define AXON_IMU_ENABLE       1
#define AXON_IMU_I2C          i2c1        // matches PIN_IMU_SDA/SCL = GP14/15
#define AXON_IMU_ADDR         0x28        // BNO055 default (ADR low)
#define AXON_IMU_BAUD         400000      // I2C fast mode
#define AXON_IMU_FRAME_ID     "imu_link"
#define AXON_IMU_TOPIC        "imu/data"
#define AXON_IMU_MAG_TOPIC    "imu/mag"
#define AXON_IMU_TEMP_TOPIC   "imu/temperature"
#define AXON_IMU_RATE_HZ      50

// Fixed diagonal covariances published with each sample (off-diagonal = 0).
// The BNO055 does not report per-axis variance, so these are nominal values.
#define AXON_IMU_ORIENT_COV   0.0159     // rad^2
#define AXON_IMU_ANGVEL_COV   0.04       // (rad/s)^2
#define AXON_IMU_LINACC_COV   0.017      // (m/s^2)^2
#define AXON_IMU_MAG_COV      0.0        // tesla^2 (0 = unknown)

// ---------------------------------------------------------------------------
// Lidar passthrough (CDC #1 <-> lidar UART, pins in pins.h)
// ---------------------------------------------------------------------------
#define AXON_LIDAR_DEFAULT_BAUD 460800   // RPLidar C1

#endif // AXON_CONFIG_H
