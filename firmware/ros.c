#include "ros.h"

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#include "cdc_serial.h"
#include "easypicoros.h"
#include "serial_hook.h"

#include "imu.h"
#include "io.h"
#include "robot.h"
#include "servos.h"
#include "status.h"
#include "usb_descriptors.h"
#include "wheels.h"

// Wheels first, then the arm. This is the slot order in joint_states, and
// it is what the host's ros2_control config expects.
#if SERVOS_ENABLED
#define ARM_JOINTS SERVO_COUNT
#else
#define ARM_JOINTS 0
#endif
#define JOINT_COUNT (WHEEL_COUNT + ARM_JOINTS)

// ===========================================================================
//  The link, and the node
// ===========================================================================

static cdc_serial_t  zenoh_cdc;
static serial_hook_t zenoh_hook;

// ===========================================================================
//  Commands in
// ===========================================================================
//
// Both command topics are Float64MultiArray, which carries variable-length
// arrays -- and easypicoros never allocates, so the message needs somewhere
// of ours to land. That is what these structs are: storage, wired up once,
// reused for every message. `n_elements` is the capacity we are offering;
// `n_deserialized` is how many actually turned up.

// Sized with headroom rather than exactly: a message carrying MORE values
// than the capacity we offer fails to deserialize outright, and the callback
// never fires. A host that sends a longer array than we use should have its
// extra values ignored, not drop the command on the floor.
#define CMD_MAX_VALUES 16

static double                  base_values[CMD_MAX_VALUES];
static ros_MultiArrayDimension base_dims[4];
static ros_Float64MultiArray   base_message = {
    .layout = { .dim = { .data = base_dims, .n_elements = 4 } },
    .data = { .data = base_values, .n_elements = CMD_MAX_VALUES },
};

// Set whenever a base_cmd arrives; read by the watchdog in ros_update() to
// notice when they stop. Starts at 0 (time since boot), which is already
// in the past the moment the main loop runs -- so a robot that never
// receives a single base_cmd is "stale" from boot, same as one that was
// receiving them and stopped. Either way, the answer is brake.
static uint64_t last_base_cmd_us = 0;

// Wheel speeds, in rad/s, in the order of the WHEELS table.
static void on_base_cmd(void *msg, void *unused) {
    (void)unused;
    ros_Float64MultiArray *cmd = msg;

    last_base_cmd_us = time_us_64();
    for (uint8_t i = 0; i < WHEEL_COUNT && i < cmd->data.n_deserialized; i++) {
        wheels_set_speed(i, cmd->data.data[i]);
    }
}

#if SERVOS_ENABLED
static double                  arm_values[CMD_MAX_VALUES];
static ros_MultiArrayDimension arm_dims[4];
static ros_Float64MultiArray   arm_message = {
    .layout = { .dim = { .data = arm_dims, .n_elements = 4 } },
    .data = { .data = arm_values, .n_elements = CMD_MAX_VALUES },
};

// Joint angles, in radians, in the order of the SERVOS table.
static void on_arm_cmd(void *msg, void *unused) {
    (void)unused;
    ros_Float64MultiArray *cmd = msg;

    for (uint8_t i = 0; i < SERVO_COUNT && i < cmd->data.n_deserialized; i++) {
        servos_set_angle(i, cmd->data.data[i]);
    }
}
#endif

// ===========================================================================
//  State out
// ===========================================================================

static easyp_publisher_t *joint_states_pub;
static easyp_publisher_t *imu_pub;
static easyp_publisher_t *imu_mag_pub;
static easyp_publisher_t *imu_temp_pub;

#if SERVOS_ENABLED && TELEMETRY_ENABLED
typedef struct {
    easyp_publisher_t *current;
    easyp_publisher_t *voltage;
    easyp_publisher_t *load;
    easyp_publisher_t *temperature;
} servo_telemetry_t;

static servo_telemetry_t telemetry[SERVO_COUNT];
#endif

// Time since boot. The board has no RTC and nothing to sync one against, so
// consumers that need wall-clock stamps re-stamp on the host.
static ros_Time now_stamp(void) {
    uint64_t us = time_us_64();
    return (ros_Time){
        .sec     = (int32_t)(us / 1000000u),
        .nanosec = (uint32_t)((us % 1000000u) * 1000u),
    };
}

// One message for every joint, every time: a wheel or servo that didn't
// answer reports zero rather than dropping out of the array, so the slots
// the host reads never move around underneath it.
static void publish_joint_states(void) {
    const char *names[JOINT_COUNT];
    double positions[JOINT_COUNT]  = {0};
    double velocities[JOINT_COUNT] = {0};
    double efforts[JOINT_COUNT]    = {0};

    for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
        names[i]     = WHEELS[i].joint;
        positions[i] = wheels_read_angle(i);
    }

#if SERVOS_ENABLED
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        uint8_t slot = WHEEL_COUNT + i;
        names[slot] = SERVOS[i].joint;
        servos_read_state(i, &positions[slot], &velocities[slot]);
    }
#endif

    ros_JointState msg = {
        .header   = { .stamp = now_stamp(), .frame_id = "" },
        .name     = { .data = (char **)names, .n_elements = JOINT_COUNT },
        .position = { .data = positions,     .n_elements = JOINT_COUNT },
        .velocity = { .data = velocities,    .n_elements = JOINT_COUNT },
        .effort   = { .data = efforts,       .n_elements = JOINT_COUNT },
    };
    easyp_publish(joint_states_pub, &msg);
}

#if SERVOS_ENABLED && TELEMETRY_ENABLED
// One servo per tick, round-robin. Reading all of them every time would put
// SERVO_COUNT extra transactions in the control loop for data nobody is
// watching at 50 Hz.
static void publish_servo_telemetry(void) {
    static uint8_t next = 0;

    uint8_t index = next;
    next = (next + 1) % SERVO_COUNT;

    st3215_telemetry_t reading;
    if (!servos_read_telemetry(index, &reading)) {
        return;
    }

    ros_Float32 value;
    value = reading.current_ma;   easyp_publish(telemetry[index].current, &value);
    value = reading.voltage_v;    easyp_publish(telemetry[index].voltage, &value);
    value = reading.load_pct;     easyp_publish(telemetry[index].load,    &value);

    ros_Int32 celsius = reading.temperature_c;
    easyp_publish(telemetry[index].temperature, &celsius);
}
#endif

// The two chips are independent, so this publishes whatever answered: the
// IMU feeds imu/data and imu/temperature, the magnetometer feeds imu/mag.
static void publish_imu(void) {
    ros_Time stamp = now_stamp();
    imu_sample_t sample;

    if (imu_read(&sample)) {
        ros_Imu imu;
        memset(&imu, 0, sizeof(imu));
        imu.header.stamp    = stamp;
        imu.header.frame_id = IMU_FRAME_ID;
        imu.angular_velocity.x = sample.gyro[0];
        imu.angular_velocity.y = sample.gyro[1];
        imu.angular_velocity.z = sample.gyro[2];
        imu.linear_acceleration.x = sample.accel[0];
        imu.linear_acceleration.y = sample.accel[1];
        imu.linear_acceleration.z = sample.accel[2];

        // -1 in the first element is how sensor_msgs/Imu says "there is no
        // orientation estimate in this message" -- the LSM6DSOX measures,
        // it does not fuse. The quaternion is left zeroed; consumers are
        // required to check this first and ignore it.
        imu.orientation_covariance[0] = -1.0;

        // Diagonal only: neither chip reports per-axis variance.
        imu.angular_velocity_covariance[0] = imu.angular_velocity_covariance[4] =
            imu.angular_velocity_covariance[8] = IMU_ANGULAR_VEL_COV;
        imu.linear_acceleration_covariance[0] = imu.linear_acceleration_covariance[4] =
            imu.linear_acceleration_covariance[8] = IMU_LINEAR_ACC_COV;
        easyp_publish(imu_pub, &imu);

        ros_Temperature temp;
        memset(&temp, 0, sizeof(temp));
        temp.header.stamp    = stamp;
        temp.header.frame_id = IMU_FRAME_ID;
        temp.temperature = (double)sample.temp_c;
        temp.variance    = 0.0;
        easyp_publish(imu_temp_pub, &temp);
    }

    float field[3];
    if (imu_read_magnetic_field(field)) {
        ros_MagneticField mag;
        memset(&mag, 0, sizeof(mag));
        mag.header.stamp    = stamp;
        mag.header.frame_id = IMU_FRAME_ID;
        mag.magnetic_field.x = field[0];
        mag.magnetic_field.y = field[1];
        mag.magnetic_field.z = field[2];
        mag.magnetic_field_covariance[0] = mag.magnetic_field_covariance[4] =
            mag.magnetic_field_covariance[8] = IMU_MAGNETIC_COV;
        easyp_publish(imu_mag_pub, &mag);
    }
}

// ===========================================================================
//  Setup
// ===========================================================================

#if SERVOS_ENABLED && TELEMETRY_ENABLED
// motor_telemetry/<joint>/current, and so on. A joint named "1" would make
// a topic name starting with a digit, which ROS does not allow, so those
// get a "joint_" in front -- same rule the host-side driver used.
//
// The names live here for good: a publisher keeps the pointer it was given
// rather than copying the string, so these buffers have to outlive it.
#define TELEMETRY_QUANTITIES 4
static char telemetry_topics[SERVO_COUNT][TELEMETRY_QUANTITIES][64];

static easyp_publisher_t *telemetry_publisher(uint8_t servo, uint8_t slot,
                                              const char *quantity) {
    const char *joint  = SERVOS[servo].joint;
    const char *prefix = (joint[0] >= '0' && joint[0] <= '9') ? "joint_" : "";
    char *topic = telemetry_topics[servo][slot];

    snprintf(topic, sizeof(telemetry_topics[servo][slot]), "%s/%s%s/%s",
             TOPIC_TELEMETRY, prefix, joint, quantity);

    // Integers for temperature, floats for everything else.
    const easyp_type_info_t *type = (strcmp(quantity, "temperature") == 0)
                                  ? EASYP_TYPE(ros_Int32)
                                  : EASYP_TYPE(ros_Float32);
    return easyp_publisher_create(topic, type);
}
#endif

bool ros_begin(void) {
    // The port zenoh talks over, wrapped so USB and the lidar keep running
    // while it waits -- including the wait below for a router that may not
    // be up yet.
    serial_t *link = serial_hook_init(&zenoh_hook,
                                      cdc_serial_init(&zenoh_cdc, CDC_IDX_ZENOH),
                                      io_poll);

    status_printf("[ros  ] connecting to the router on '%s'...\n", ZENOH_LOCATOR);
    easyp_node_config_t node = { .name = NODE_NAME, .domain_id = ROS_DOMAIN_ID };
    if (!easyp_node_init(node, ZENOH_MODE, ZENOH_LOCATOR, link)) {
        return false;
    }
    status_printf("[ros  ] session up; declaring '%s'\n", NODE_NAME);

    joint_states_pub = easyp_publisher_create(TOPIC_JOINT_STATES, EASYP_TYPE(ros_JointState));
    imu_pub          = easyp_publisher_create(TOPIC_IMU,          EASYP_TYPE(ros_Imu));
    imu_mag_pub      = easyp_publisher_create(TOPIC_IMU_MAG,      EASYP_TYPE(ros_MagneticField));
    imu_temp_pub     = easyp_publisher_create(TOPIC_IMU_TEMP,     EASYP_TYPE(ros_Temperature));
    if (!joint_states_pub || !imu_pub || !imu_mag_pub || !imu_temp_pub) {
        return false;
    }

    if (!easyp_subscriber_create_into(TOPIC_BASE_CMD, EASYP_TYPE(ros_Float64MultiArray),
                                      on_base_cmd, NULL, &base_message)) {
        return false;
    }

#if SERVOS_ENABLED
    if (!easyp_subscriber_create_into(TOPIC_ARM_CMD, EASYP_TYPE(ros_Float64MultiArray),
                                      on_arm_cmd, NULL, &arm_message)) {
        return false;
    }
#if TELEMETRY_ENABLED
    // Declared for every servo, online or not, so the topic list doesn't
    // depend on what happened to be plugged in at boot.
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        telemetry[i].current     = telemetry_publisher(i, 0, "current");
        telemetry[i].voltage     = telemetry_publisher(i, 1, "voltage");
        telemetry[i].load        = telemetry_publisher(i, 2, "load");
        telemetry[i].temperature = telemetry_publisher(i, 3, "temperature");
    }
#endif
#endif

    status_printf("[ros  ] %u joints, IMU %s%s\n", JOINT_COUNT,
                  imu_online() ? "online" : "offline",
                  ARM_JOINTS ? ", arm topics live" : ", no arm");
    return true;
}

// ===========================================================================
//  The loop
// ===========================================================================

// Has this deadline passed? Advances it if so, so each job free-runs at its
// own rate without drifting into the others.
static bool due(uint64_t *deadline_us, uint32_t hz) {
    uint64_t now = time_us_64();
    if (now < *deadline_us) {
        return false;
    }
    *deadline_us = now + 1000000u / hz;
    return true;
}

// Command watchdog: base_cmd going quiet for longer than COMMAND_TIMEOUT_MS
// means the host stalled, crashed, or lost the link -- not that it wants
// the wheels to keep doing whatever they were last told forever. Brake
// keeps being re-sent at COMMAND_WATCHDOG_HZ for as long as the silence
// lasts, so one dropped frame doesn't leave a wheel spinning.
static void base_cmd_watchdog(void) {
    static uint64_t check_due = 0;

    if (!due(&check_due, COMMAND_WATCHDOG_HZ)) {
        return;
    }
    if (time_us_64() - last_base_cmd_us < (uint64_t)COMMAND_TIMEOUT_MS * 1000) {
        return;   // heard from the host recently enough
    }
    for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
        wheels_brake(i);
    }
}

void ros_update(void) {
    static uint64_t joint_states_due = 0;
    static uint64_t telemetry_due    = 0;
    static uint64_t imu_due          = 0;

    // Receive first: command callbacks fire from in here.
    easyp_spin_once();

    base_cmd_watchdog();

    if (due(&joint_states_due, JOINT_STATES_HZ)) {
        publish_joint_states();
    }
#if SERVOS_ENABLED && TELEMETRY_ENABLED
    if (due(&telemetry_due, TELEMETRY_HZ)) {
        publish_servo_telemetry();
    }
#else
    (void)telemetry_due;
#endif
    if (due(&imu_due, IMU_HZ)) {
        publish_imu();
    }
}
