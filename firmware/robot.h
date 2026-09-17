/*
 * base101 — everything you might want to change.
 * ==============================================
 *
 * This is the whole configuration of the robot: what is wired where, which
 * motor is which wheel, which way each one spins, what gets published and
 * how often. Nothing else in the firmware has numbers like these in it, so
 * if you are retuning, re-wiring or renaming a joint, you are in the right
 * file — edit it and reflash.
 *
 * Board pins that are soldered down (the servo bus, I2C, the LED strip) are
 * not here: they live in link101/pins.h, because they are facts about the
 * board rather than choices about the robot. What IS here is everything
 * plugged into the breakout headers.
 */

#ifndef ROBOT_H
#define ROBOT_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/i2c.h"
#include "link101/pins.h"

// ===========================================================================
//  Wiring
// ===========================================================================

// RPLidar C1 on the uart1 hardware peripheral.
#define LIDAR_TX_PIN    4
#define LIDAR_RX_PIN    5
#define LIDAR_BAUD      460800      // RPLidar C1 default

// The IMU soldered to the board: an LSM6DSOX (accel + gyro) and an
// MMC5983MA magnetometer, both on i2c1 — the same bus as the Qwiic
// connector. Addresses are fixed by the layout; see link101/pins.h.
#define IMU_I2C         i2c1
#define IMU_ADDR        LINK101_LSM6DSOX_I2C_ADDR   // 0x6B
#define MAG_ADDR        LINK101_MMC5983_I2C_ADDR    // 0x30
#define IMU_I2C_BAUD    400000      // fast mode

// Feetech STS/SCS servo bus. Half duplex on the board's fixed pins; the
// servos are told apart by ID, not by wire.
#define SERVO_BAUD      1000000

// Wheel motors. Each DDSM210 needs its own UART -- it cannot share a TX
// line without external gating -- so this is four PIO UARTs on the
// breakout headers. Lower pin is our TX (to the motor's RX).
#define WHEEL_BAUD      115200

// ===========================================================================
//  The wheels
// ===========================================================================
//
// Four independent wheels, commanded as a 4-element array of rad/s on
// base_cmd. Left wheels run direction -1 because they face the other way.
//
// Every motor keeps the factory ID 1: each one is alone on its own bus, so
// the IDs never collide. Only change an id here if you re-addressed a motor.
//
// If a wheel stays silent, swap its tx/rx -- connector wiring varies.

#define WHEEL_COUNT 4

typedef struct {
    uint8_t     tx_pin, rx_pin;
    uint8_t     id;
    const char *joint;
    int8_t      direction;   // +1 or -1
} wheel_cfg_t;

static const wheel_cfg_t WHEELS[WHEEL_COUNT] = {
    { 21, 22, 1, "front_left_wheel_joint",  -1 },
    { 19, 20, 1, "front_right_wheel_joint", +1 },
    { 25, 26, 1, "back_left_wheel_joint",   -1 },
    { 23, 24, 1, "back_right_wheel_joint",  +1 },
};

// The order above IS the order of the base_cmd array and of the wheel slots
// in joint_states. Reorder the rows and both follow.

#define WHEEL_MAX_RPM     200    // command cap; the hardware ceiling is 210
#define WHEEL_ACCEL_TIME  20     // the DDSM210's own ramp, 0.1 ms per RPM of change
#define WHEEL_SPEED_SCALE 1.0    // fudge factor on commanded speed

// base_cmd can step from one speed to a very different one between
// messages -- reversing direction, or a sharp turn asking the two sides
// for very different speeds -- and a wheel commanded straight to that new
// target skids across the floor getting there instead of tracking it.
// wheels_update() ramps the ACTUAL commanded speed toward whatever
// base_cmd last asked for, at most this many rad/s of change per second,
// regardless of how big the step in the command was. Lower feels gentler
// and drags less on stops and turns; higher tracks the host more closely.
#define WHEEL_ACCEL_LIMIT_RAD_S2   6.0

// How often the ramp advances and (if it moved) sends a fresh speed --
// matches the other control-loop rates below.
#define WHEEL_CONTROL_HZ           50

// ===========================================================================
//  The arm
// ===========================================================================
//
// Feetech servos in position mode, commanded as an array of radians on
// arm_cmd in the order listed here. Set SERVOS_ENABLED to false for a
// wheels-only robot -- the arm topics then don't appear at all.
//
// These used to be editable at runtime over a JSON console. They are
// compiled in now: an arm you can re-map without reflashing turned out to
// be a feature nobody used and a lot of firmware to carry.

#define SERVOS_ENABLED true
#define SERVO_COUNT    6

typedef struct {
    uint8_t     id;         // Feetech bus ID
    const char *joint;      // joint name, and the telemetry topic token
    int8_t      direction;  // +1 or -1
} servo_cfg_t;

static const servo_cfg_t SERVOS[SERVO_COUNT] = {
    { 1, "1", +1 },
    { 2, "2", +1 },
    { 3, "3", +1 },
    { 4, "4", +1 },
    { 5, "5", +1 },
    { 6, "6", +1 },
};

#define SERVO_TICKS_PER_REV   4096   // 0..4095 over one full turn
#define SERVO_MOVE_SPEED      200    // steps/s for a position move
#define SERVO_MOVE_ACCEL      200    // acceleration for a position move
#define SERVO_DEFAULT_ACCEL   20     // written to each servo at startup
#define SERVO_SPEED_SCALE     0.5    // scales both of the move numbers above

// ===========================================================================
//  ROS
// ===========================================================================
//
// Topic names and the node name have to match what the host expects
// (base101_control_plugin and the ros2_control config), so treat these as a
// shared interface rather than a preference.

#define NODE_NAME        "axon"
#define ROS_DOMAIN_ID    0
#define ZENOH_MODE       "client"
#define ZENOH_LOCATOR    "serial/cdc#baudrate=921600"

#define TOPIC_BASE_CMD       "motor_manager/base_cmd"       // 4 wheel speeds, rad/s
#define TOPIC_ARM_CMD        "motor_manager/arm_cmd"        // arm angles, rad
#define TOPIC_JOINT_STATES   "motor_manager/joint_states"   // wheels + arm
#define TOPIC_TELEMETRY      "motor_telemetry"              // prefix: <it>/<joint>/current etc
#define TOPIC_IMU            "imu/data"
#define TOPIC_IMU_MAG        "imu/mag"
#define TOPIC_IMU_TEMP       "imu/temperature"

#define IMU_FRAME_ID     "imu_link"

// How often each thing is published. Everything is sampled from the main
// loop, so these are ceilings, not guarantees -- a slow bus read pushes the
// whole loop out.
#define JOINT_STATES_HZ  50
#define TELEMETRY_HZ     50     // one servo per tick, round-robin
#define IMU_HZ           50

#define TELEMETRY_ENABLED true

// If nothing fresh arrives on base_cmd within this long, the wheels are
// actively braked instead of left spinning at whatever they were last
// told -- a host that stalls, crashes, or drops the ROS link mid-drive
// should not leave the robot coasting on a stale command forever. Checked
// at COMMAND_WATCHDOG_HZ, which also bounds how often a brake frame is
// re-sent while the link stays down -- cheap insurance against the one
// frame that mattered getting lost.
//
// The arm needs no equivalent: it is position-controlled, and a Feetech
// servo already holds its last commanded position against gravity and
// friction on its own, with or without new commands arriving.
#define COMMAND_TIMEOUT_MS     500
#define COMMAND_WATCHDOG_HZ    5

// Neither chip reports per-axis variance, so these nominal diagonals go out
// with every sample. Off-diagonal terms are zero.
//
// There is no orientation covariance because there is no orientation: the
// LSM6DSOX does no fusion, so /imu/data sets orientation_covariance[0] = -1,
// which is how sensor_msgs/Imu says "no orientation estimate". Fuse on the
// host (imu_filter_madgwick, robot_localization) from imu/data + imu/mag.
#define IMU_ANGULAR_VEL_COV  0.04     // (rad/s)^2
#define IMU_LINEAR_ACC_COV   0.017    // (m/s^2)^2
#define IMU_MAGNETIC_COV     0.0      // tesla^2 (0 = unknown)

// ===========================================================================
//  Status LED
// ===========================================================================
//
// The six-pixel strip says what the firmware is doing, from across the room:
//
//   fast yellow blink   waiting for the ROS router -- nothing else can start
//   slow green breath   connected, running
//
// Either way it is driven from the main loop, so a strip that stops moving
// means a loop that stopped turning. That is half the point of it.

#define LED_ENABLED     true

// Waiting: a blink, not a breath. It should read as "waiting on you".
#define LED_WAITING_PERIOD_MS 400    // one on/off cycle
#define LED_WAITING_R         40     // yellow, weighted warm -- the green
#define LED_WAITING_G         26     // die is the brighter of the two

// Connected: a slow breath, calm enough to ignore.
#define LED_READY_PERIOD_MS   3000   // one full breath, in and out
#define LED_MIN               3      // channel value at the dimmest point
#define LED_MAX               40     // and at the brightest (kept low on purpose)

#endif // ROBOT_H
