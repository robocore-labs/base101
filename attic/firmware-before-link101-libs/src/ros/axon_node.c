#include "axon_node.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include "picoros.h"
#include "picoserdes.h"

#include "pins.h"
#include "../dbg.h"
#include "axon_config.h"
#include "../config/axon_cfg.h"
#include "../motors/ddsm210.h"
#include "../motors/st3215.h"
#include "../sensors/bno055.h"

#define TWO_PI (2.0 * M_PI)

// Max JointState slots: the fixed wheels + the runtime-configurable servos.
#define AXON_JOINT_MAX (AXON_DDSM_COUNT + AXON_SERVO_MAX)

// ---------------------------------------------------------------------------
// Servo table, built at init from the runtime config (g_cfg). Each entry
// carries its own copy of the mapping so nothing points back into config.
// ---------------------------------------------------------------------------
typedef struct {
    uint8_t  id;
    char     joint[AXON_JOINT_NAME_LEN];
    int8_t   direction;
    uint8_t  command_index;   // index into arm_cmd array
    uint8_t  state_index;     // slot in JointState arrays
    uint8_t  accel;
    bool     online;
} st_motor_t;

static st_motor_t st_motors[AXON_SERVO_MAX];
static uint8_t st_motor_count = 0;

static bool ddsm_online[AXON_DDSM_COUNT];

// ---------------------------------------------------------------------------
// Node + endpoints
// ---------------------------------------------------------------------------
static picoros_node_t node = {
    .name = AXON_NODE_NAME,
    .domain_id = AXON_ROS_DOMAIN_ID,
};

static picoros_publisher_t pub_joint_states = {
    .topic = {
        .name = AXON_JOINT_STATES_TOPIC,
        .type = ROSTYPE_NAME(ros_JointState),
        .rihs_hash = ROSTYPE_HASH(ros_JointState),
    },
};

// Liveness counters surfaced to the main-loop heartbeat (axon_node_status).
static volatile uint32_t base_cmd_rx = 0;
static volatile uint32_t arm_cmd_rx = 0;
static uint32_t joint_pub_count = 0;
static uint32_t imu_pub_count = 0;

void axon_node_status(uint32_t *base_cmds, uint32_t *arm_cmds,
                      uint32_t *joint_pubs, uint32_t *imu_pubs) {
    if (base_cmds)  *base_cmds  = base_cmd_rx;
    if (arm_cmds)   *arm_cmds   = arm_cmd_rx;
    if (joint_pubs) *joint_pubs = joint_pub_count;
    if (imu_pubs)   *imu_pubs   = imu_pub_count;
}

#if AXON_IMU_ENABLE
static bool imu_online = false;

static picoros_publisher_t pub_imu = {
    .topic = {
        .name = AXON_IMU_TOPIC,
        .type = ROSTYPE_NAME(ros_Imu),
        .rihs_hash = ROSTYPE_HASH(ros_Imu),
    },
};
static picoros_publisher_t pub_mag = {
    .topic = {
        .name = AXON_IMU_MAG_TOPIC,
        .type = ROSTYPE_NAME(ros_MagneticField),
        .rihs_hash = ROSTYPE_HASH(ros_MagneticField),
    },
};
static picoros_publisher_t pub_temp = {
    .topic = {
        .name = AXON_IMU_TEMP_TOPIC,
        .type = ROSTYPE_NAME(ros_Temperature),
        .rihs_hash = ROSTYPE_HASH(ros_Temperature),
    },
};
#endif

#if AXON_MOTOR_TELEMETRY_ENABLE
typedef struct {
    char current_topic[64];
    char voltage_topic[64];
    char load_topic[64];
    char temperature_topic[64];
    picoros_publisher_t current;
    picoros_publisher_t voltage;
    picoros_publisher_t load;
    picoros_publisher_t temperature;
} st_telemetry_pubs_t;

static st_telemetry_pubs_t st_telemetry_pubs[AXON_SERVO_MAX];
#endif

static picoros_subscriber_t sub_base_cmd;
static picoros_subscriber_t sub_arm_cmd;

static uint8_t pub_buf[1536];

// ---------------------------------------------------------------------------
// Float64MultiArray command parsing
// ---------------------------------------------------------------------------
#define CMD_MAX_VALUES 16

static uint32_t parse_cmd(uint8_t *rx_data, size_t len, double *out, uint32_t cap) {
    ros_MultiArrayDimension dims[4];
    memset(dims, 0, sizeof(dims));

    ros_Float64MultiArray msg;
    memset(&msg, 0, sizeof(msg));
    msg.layout.dim.data = dims;
    msg.layout.dim.n_elements = 4;
    msg.data.data = out;
    msg.data.n_elements = cap;

    if (!ps_deserialize(rx_data, &msg, len)) {
        return 0;
    }
    return msg.data.n_deserialized;
}

static bool values_changed(const double *vals, double *last, uint32_t n, bool *have_last) {
    bool changed = !*have_last;
    if (*have_last) {
        for (uint32_t i = 0; i < n; i++) {
            if (fabs(vals[i] - last[i]) >= 1e-6) {
                changed = true;
                break;
            }
        }
    }
    if (changed) {
        memcpy(last, vals, n * sizeof(double));
        *have_last = true;
    }
    return changed;
}

// ---------------------------------------------------------------------------
// DDSM210 base command (ddsm210_node._handle_command port)
// ---------------------------------------------------------------------------
static void on_base_cmd(uint8_t *rx_data, size_t data_len) {
    static double last_vals[AXON_DDSM_COUNT];
    static bool have_last = false;

    if (++base_cmd_rx == 1) {
        dbg_printf("[ros ] first base_cmd received (%u bytes)\n", (unsigned)data_len);
    }

    double data[CMD_MAX_VALUES];
    uint32_t n = parse_cmd(rx_data, data_len, data, CMD_MAX_VALUES);

    double vals[AXON_DDSM_COUNT];
    for (int i = 0; i < AXON_DDSM_COUNT; i++) {
        if (AXON_DDSM_MOTORS[i].command_index >= n) {
            return;  // command too short
        }
        vals[i] = data[AXON_DDSM_MOTORS[i].command_index];
    }

    if (!values_changed(vals, last_vals, AXON_DDSM_COUNT, &have_last)) {
        return;
    }

    for (int i = 0; i < AXON_DDSM_COUNT; i++) {
        const axon_ddsm_motor_cfg_t *m = &AXON_DDSM_MOTORS[i];
        double rad_per_sec = vals[i] * m->direction;
        double rpm = rad_per_sec * AXON_DDSM_SPEED_SCALE * 60.0 / TWO_PI;
        int32_t rpm_x10 = (int32_t)(rpm * 10.0);

        int32_t cap = AXON_DDSM_MAX_RPM * 10;
        if (cap > DDSM210_MAX_SPEED_RAW) cap = DDSM210_MAX_SPEED_RAW;
        if (rpm_x10 > cap) rpm_x10 = cap;
        if (rpm_x10 < -cap) rpm_x10 = -cap;

        ddsm210_set_velocity(m->port, m->motor_id, (int16_t)rpm_x10, AXON_DDSM_ACCEL_TIME, NULL);
    }
}

// ---------------------------------------------------------------------------
// ST3215 servo command (arm group, position mode)
// ---------------------------------------------------------------------------
static void st_send_position(const st_motor_t *m, double angle_rad) {
    double ticks_f = (double)(AXON_ST_TICKS_PER_REV / 2) + (angle_rad / TWO_PI) * AXON_ST_TICKS_PER_REV;
    int32_t ticks = (int32_t)ticks_f;
    if (ticks < 0) ticks = 0;
    if (ticks > AXON_ST_TICKS_PER_REV - 1) ticks = AXON_ST_TICKS_PER_REV - 1;

    uint16_t speed = (uint16_t)((double)AXON_SERVO_POSITION_SPEED * AXON_SERVO_SPEED_SCALE);
    uint8_t accel = (uint8_t)((double)AXON_SERVO_POSITION_ACCEL * AXON_SERVO_SPEED_SCALE);
    st3215_move_to(m->id, (uint16_t)ticks, speed, accel);
}

static void on_arm_cmd(uint8_t *rx_data, size_t data_len) {
    static double last_vals[AXON_SERVO_MAX];
    static bool have_last = false;

    if (++arm_cmd_rx == 1) {
        dbg_printf("[ros ] first arm_cmd received (%u bytes)\n", (unsigned)data_len);
    }

    if (st_motor_count == 0) {
        return;
    }

    double data[CMD_MAX_VALUES];
    uint32_t n = parse_cmd(rx_data, data_len, data, CMD_MAX_VALUES);

    double vals[AXON_SERVO_MAX];
    for (uint8_t i = 0; i < st_motor_count; i++) {
        if (st_motors[i].command_index >= n) {
            return;  // command too short
        }
        vals[i] = data[st_motors[i].command_index];
    }

    if (!values_changed(vals, last_vals, st_motor_count, &have_last)) {
        return;
    }

    for (uint8_t i = 0; i < st_motor_count; i++) {
        if (!st_motors[i].online) {
            continue;
        }
        st_send_position(&st_motors[i], vals[i] * st_motors[i].direction);
    }
}

// ---------------------------------------------------------------------------
// Motor initialization (mode setup, torque, hold position)
// ---------------------------------------------------------------------------
void axon_node_motors_init(void) {
    // DDSM210 wheels: verify connectivity, switch to velocity loop.
    for (int i = 0; i < AXON_DDSM_COUNT; i++) {
        const axon_ddsm_motor_cfg_t *m = &AXON_DDSM_MOTORS[i];
        int mode = ddsm210_get_mode(m->port, m->motor_id);
        ddsm_online[i] = (mode >= 0);
        dbg_printf("[axon] DDSM %u (%s): %s\n", m->motor_id, m->joint_name,
               ddsm_online[i] ? "online" : "no response");
        ddsm210_set_mode(m->port, m->motor_id, DDSM210_MODE_VELOCITY);
        sleep_ms(10);
    }

    // ST3215 servos: build the table from runtime config, ping, set position
    // mode. Every configured (per-servo-enabled) servo is included regardless
    // of the advisory g_cfg.servos_enabled flag; offline servos stay in the
    // table (marked offline) so their joints/topics still exist.
    st_motor_count = 0;
    for (uint8_t i = 0; i < g_cfg.servo_count && st_motor_count < AXON_SERVO_MAX; i++) {
        if (!g_cfg.servos[i].enable) {
            continue;
        }
        st_motor_t *m = &st_motors[st_motor_count];
        memset(m, 0, sizeof(*m));
        m->id = g_cfg.servos[i].id;
        strncpy(m->joint, g_cfg.servos[i].joint, sizeof(m->joint) - 1);
        m->direction = 1;
        m->command_index = st_motor_count;
        m->state_index = (uint8_t)(AXON_DDSM_COUNT + st_motor_count);
        m->accel = AXON_SERVO_DEFAULT_ACCEL;
        st_motor_count++;

        m->online = st3215_ping(m->id);
        dbg_printf("[axon] ST3215 %u (%s): %s\n", m->id, m->joint,
                   m->online ? "online" : "offline");
        if (!m->online) {
            continue;
        }

        st3215_set_mode(m->id, ST3215_MODE_POSITION);
        st3215_set_acceleration(m->id, m->accel);
        int32_t pos = st3215_read_position(m->id);
        st3215_set_torque(m->id, true);
        if (pos >= 0) {
            // Hold the current position (gentle move like the original manager).
            st3215_move_to(m->id, (uint16_t)pos, 100, m->accel);
        }
        sleep_ms(10);
    }
}

void axon_imu_init(void) {
#if AXON_IMU_ENABLE
    i2c_init(AXON_IMU_I2C, AXON_IMU_BAUD);
    gpio_set_function(PIN_IMU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_IMU_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_IMU_SDA);
    gpio_pull_up(PIN_IMU_SCL);

    imu_online = bno055_init(AXON_IMU_I2C, AXON_IMU_ADDR);
    dbg_printf("[axon] BNO055 IMU: %s\n", imu_online ? "online" : "no response");
#endif
}

// ---------------------------------------------------------------------------
// Declarations
// ---------------------------------------------------------------------------
#if AXON_MOTOR_TELEMETRY_ENABLE
static void declare_st_telemetry_pubs(void) {
    for (uint8_t i = 0; i < st_motor_count; i++) {
        // Declare telemetry topics for every configured servo, online or not,
        // so the topics always exist (offline ones simply publish nothing).
        st_telemetry_pubs_t *p = &st_telemetry_pubs[i];
        const char *name = st_motors[i].joint;
        // ROS topic tokens can't start with a digit
        const char *prefix = (name[0] >= '0' && name[0] <= '9') ? "joint_" : "";

        snprintf(p->current_topic, sizeof(p->current_topic), "%s/%s%s/current",
                 AXON_MOTOR_TELEMETRY_PREFIX, prefix, name);
        snprintf(p->voltage_topic, sizeof(p->voltage_topic), "%s/%s%s/voltage",
                 AXON_MOTOR_TELEMETRY_PREFIX, prefix, name);
        snprintf(p->load_topic, sizeof(p->load_topic), "%s/%s%s/load",
                 AXON_MOTOR_TELEMETRY_PREFIX, prefix, name);
        snprintf(p->temperature_topic, sizeof(p->temperature_topic), "%s/%s%s/temperature",
                 AXON_MOTOR_TELEMETRY_PREFIX, prefix, name);

        p->current.topic.name = p->current_topic;
        p->current.topic.type = ROSTYPE_NAME(ros_Float32);
        p->current.topic.rihs_hash = ROSTYPE_HASH(ros_Float32);
        picoros_publisher_declare(&node, &p->current);

        p->voltage.topic.name = p->voltage_topic;
        p->voltage.topic.type = ROSTYPE_NAME(ros_Float32);
        p->voltage.topic.rihs_hash = ROSTYPE_HASH(ros_Float32);
        picoros_publisher_declare(&node, &p->voltage);

        p->load.topic.name = p->load_topic;
        p->load.topic.type = ROSTYPE_NAME(ros_Float32);
        p->load.topic.rihs_hash = ROSTYPE_HASH(ros_Float32);
        picoros_publisher_declare(&node, &p->load);

        p->temperature.topic.name = p->temperature_topic;
        p->temperature.topic.type = ROSTYPE_NAME(ros_Int32);
        p->temperature.topic.rihs_hash = ROSTYPE_HASH(ros_Int32);
        picoros_publisher_declare(&node, &p->temperature);
    }
}
#endif

bool axon_node_declare(void) {
    if (picoros_node_init(&node) != PICOROS_OK) {
        return false;
    }

    if (picoros_publisher_declare(&node, &pub_joint_states) != PICOROS_OK) {
        return false;
    }

#if AXON_IMU_ENABLE
    // Declare the IMU topics whether or not the BNO055 answered at boot, so the
    // topics always exist. When offline we simply publish nothing (no fake data).
    if (picoros_publisher_declare(&node, &pub_imu) != PICOROS_OK ||
        picoros_publisher_declare(&node, &pub_mag) != PICOROS_OK ||
        picoros_publisher_declare(&node, &pub_temp) != PICOROS_OK) {
        return false;
    }
#endif

    sub_base_cmd.topic.name = AXON_BASE_CMD_TOPIC;
    sub_base_cmd.topic.type = ROSTYPE_NAME(ros_Float64MultiArray);
    sub_base_cmd.topic.rihs_hash = ROSTYPE_HASH(ros_Float64MultiArray);
    sub_base_cmd.user_callback = on_base_cmd;
    if (picoros_subscriber_declare(&node, &sub_base_cmd) != PICOROS_OK) {
        return false;
    }

    // The arm command + servo telemetry only exist when servos are enabled.
    if (st_motor_count > 0) {
        sub_arm_cmd.topic.name = AXON_ARM_CMD_TOPIC;
        sub_arm_cmd.topic.type = ROSTYPE_NAME(ros_Float64MultiArray);
        sub_arm_cmd.topic.rihs_hash = ROSTYPE_HASH(ros_Float64MultiArray);
        sub_arm_cmd.user_callback = on_arm_cmd;
        if (picoros_subscriber_declare(&node, &sub_arm_cmd) != PICOROS_OK) {
            return false;
        }
#if AXON_MOTOR_TELEMETRY_ENABLE
        declare_st_telemetry_pubs();
#endif
    }

    dbg_printf("[ros ] declared: joint_states + imu/mag/temperature (IMU %s) ; "
               "sub base_cmd%s ; %u servos configured\n",
               imu_online ? "online" : "OFFLINE",
               st_motor_count > 0 ? " + arm_cmd + telemetry" : "",
               (unsigned)st_motor_count);
    return true;
}

// ---------------------------------------------------------------------------
// Telemetry
// ---------------------------------------------------------------------------
static void publish_joint_states(void) {
    const char *names[AXON_JOINT_MAX];
    double positions[AXON_JOINT_MAX] = {0};
    double velocities[AXON_JOINT_MAX] = {0};
    double efforts[AXON_JOINT_MAX] = {0};

    // Active slots: the 4 wheels plus however many servos are configured.
    uint32_t joint_count = AXON_DDSM_COUNT + st_motor_count;
    for (uint32_t i = 0; i < joint_count; i++) {
        names[i] = "";
    }

    // DDSM210 wheels: absolute multi-turn position from mileage + encoder.
    // Every wheel slot is always named; a non-responding wheel reports zeros.
    for (int i = 0; i < AXON_DDSM_COUNT; i++) {
        const axon_ddsm_motor_cfg_t *m = &AXON_DDSM_MOTORS[i];
        names[m->state_index] = m->joint_name;
        ddsm210_odometry_t odom;
        if (ddsm210_get_odometry(m->port, m->motor_id, &odom)) {
            double fractional = ((double)odom.position / DDSM210_ENCODER_TICKS) * TWO_PI;
            double total = (double)odom.mileage_laps * TWO_PI + fractional;
            positions[m->state_index] = total * m->direction;
        }
        // offline / read failure -> leave position & velocity at zero
    }

    // ST3215 servos. Every configured servo slot is always named; an offline
    // or non-responding servo reports zeros.
    double steps_per_radian = (double)AXON_ST_TICKS_PER_REV / TWO_PI;
    for (uint8_t i = 0; i < st_motor_count; i++) {
        st_motor_t *m = &st_motors[i];
        uint8_t si = m->state_index;
        if (si >= joint_count) {
            continue;
        }
        names[si] = m->joint;
        uint16_t raw_pos;
        int16_t speed;
        if (m->online && st3215_read_state(m->id, &raw_pos, &speed)) {
            positions[si] = ((double)((int32_t)raw_pos - AXON_ST_TICKS_PER_REV / 2) /
                             AXON_ST_TICKS_PER_REV) * TWO_PI * m->direction;
            velocities[si] = ((double)speed / steps_per_radian) * m->direction;
        }
        // offline / read failure -> leave position & velocity at zero
    }

    // Always publish — joint_states stays alive even with nothing responding.
    uint64_t now_us = time_us_64();
    ros_JointState msg = {
        .header = {
            .stamp = {
                .sec = (int32_t)(now_us / 1000000u),
                .nanosec = (uint32_t)((now_us % 1000000u) * 1000u),
            },
            .frame_id = "",
        },
        .name = {.data = (char **)names, .n_elements = joint_count},
        .position = {.data = positions, .n_elements = joint_count},
        .velocity = {.data = velocities, .n_elements = joint_count},
        .effort = {.data = efforts, .n_elements = joint_count},
    };

    size_t len = ps_serialize(pub_buf, &msg, sizeof(pub_buf));
    if (len > 0) {
        picoros_publish(&pub_joint_states, pub_buf, len);
        joint_pub_count++;
    }
}

#if AXON_MOTOR_TELEMETRY_ENABLE
// One servo per tick (round-robin) keeps bus time per cycle bounded.
static void publish_motor_telemetry(void) {
    static uint8_t rr = 0;

    if (st_motor_count == 0) {
        return;
    }
    rr = (uint8_t)((rr + 1) % st_motor_count);
    st_motor_t *m = &st_motors[rr];
    if (!m->online) {
        return;
    }

    st3215_telemetry_t t;
    if (!st3215_read_telemetry(m->id, &t)) {
        return;
    }

    st_telemetry_pubs_t *p = &st_telemetry_pubs[rr];
    uint8_t buf[16];
    size_t len;

    ros_Float32 f = t.current_ma;
    len = ps_serialize(buf, &f, sizeof(buf));
    if (len > 0) picoros_publish(&p->current, buf, len);

    f = t.voltage_v;
    len = ps_serialize(buf, &f, sizeof(buf));
    if (len > 0) picoros_publish(&p->voltage, buf, len);

    f = t.load_pct;
    len = ps_serialize(buf, &f, sizeof(buf));
    if (len > 0) picoros_publish(&p->load, buf, len);

    ros_Int32 temp = t.temperature_c;
    len = ps_serialize(buf, &temp, sizeof(buf));
    if (len > 0) picoros_publish(&p->temperature, buf, len);
}
#endif

#if AXON_IMU_ENABLE
// Publish one BNO055 sample as Imu + MagneticField + Temperature.
static void publish_imu(void) {
    if (!imu_online) {
        return;
    }

    bno055_sample_t s;
    if (!bno055_read(&s)) {
        return;
    }

    uint64_t now_us = time_us_64();
    ros_Time stamp = {
        .sec = (int32_t)(now_us / 1000000u),
        .nanosec = (uint32_t)((now_us % 1000000u) * 1000u),
    };
    size_t len;

    ros_Imu imu;
    memset(&imu, 0, sizeof(imu));
    imu.header.stamp = stamp;
    imu.header.frame_id = AXON_IMU_FRAME_ID;
    imu.orientation.w = s.quat[0];
    imu.orientation.x = s.quat[1];
    imu.orientation.y = s.quat[2];
    imu.orientation.z = s.quat[3];
    imu.angular_velocity.x = s.gyro[0];
    imu.angular_velocity.y = s.gyro[1];
    imu.angular_velocity.z = s.gyro[2];
    imu.linear_acceleration.x = s.accel[0];
    imu.linear_acceleration.y = s.accel[1];
    imu.linear_acceleration.z = s.accel[2];
    imu.orientation_covariance[0] = imu.orientation_covariance[4] =
        imu.orientation_covariance[8] = AXON_IMU_ORIENT_COV;
    imu.angular_velocity_covariance[0] = imu.angular_velocity_covariance[4] =
        imu.angular_velocity_covariance[8] = AXON_IMU_ANGVEL_COV;
    imu.linear_acceleration_covariance[0] = imu.linear_acceleration_covariance[4] =
        imu.linear_acceleration_covariance[8] = AXON_IMU_LINACC_COV;
    len = ps_serialize(pub_buf, &imu, sizeof(pub_buf));
    if (len > 0) {
        picoros_publish(&pub_imu, pub_buf, len);
        imu_pub_count++;
    }

    ros_MagneticField mag;
    memset(&mag, 0, sizeof(mag));
    mag.header.stamp = stamp;
    mag.header.frame_id = AXON_IMU_FRAME_ID;
    mag.magnetic_field.x = s.mag[0];
    mag.magnetic_field.y = s.mag[1];
    mag.magnetic_field.z = s.mag[2];
    mag.magnetic_field_covariance[0] = mag.magnetic_field_covariance[4] =
        mag.magnetic_field_covariance[8] = AXON_IMU_MAG_COV;
    len = ps_serialize(pub_buf, &mag, sizeof(pub_buf));
    if (len > 0) picoros_publish(&pub_mag, pub_buf, len);

    ros_Temperature temp;
    memset(&temp, 0, sizeof(temp));
    temp.header.stamp = stamp;
    temp.header.frame_id = AXON_IMU_FRAME_ID;
    temp.temperature = (double)s.temp_c;
    temp.variance = 0.0;
    len = ps_serialize(pub_buf, &temp, sizeof(pub_buf));
    if (len > 0) picoros_publish(&pub_temp, pub_buf, len);
}
#endif

void axon_node_spin(void) {
    static uint64_t next_joint_states_us = 0;
    static uint64_t next_motor_telemetry_us = 0;
#if AXON_IMU_ENABLE
    static uint64_t next_imu_us = 0;
#endif

    uint64_t now = time_us_64();

    if (now >= next_joint_states_us) {
        next_joint_states_us = now + (1000000u / AXON_JOINT_STATES_RATE_HZ);
        publish_joint_states();
    }

#if AXON_MOTOR_TELEMETRY_ENABLE
    if (now >= next_motor_telemetry_us) {
        next_motor_telemetry_us = now + (1000000u / AXON_MOTOR_TELEMETRY_RATE_HZ);
        publish_motor_telemetry();
    }
#endif

#if AXON_IMU_ENABLE
    if (now >= next_imu_us) {
        next_imu_us = now + (1000000u / AXON_IMU_RATE_HZ);
        publish_imu();
    }
#endif
}
