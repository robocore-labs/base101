#include "servos.h"

#include <math.h>

#include "pico/stdlib.h"

#include "link101/buses.h"
#include "serial_hook.h"

#include "io.h"
#include "robot.h"
#include "status.h"

#define TWO_PI      6.283185307179586
#define CENTRE_TICK (SERVO_TICKS_PER_REV / 2)

static link101_pio_serial_t port;
static serial_hook_t        hook;
static serial_t            *bus;
static bool                 online[SERVO_COUNT];
static double               last_command[SERVO_COUNT];

// Angle in radians (0 = centre) <-> the servo's 0..4095 tick scale.
static uint16_t radians_to_ticks(double radians) {
    double ticks = CENTRE_TICK + (radians / TWO_PI) * SERVO_TICKS_PER_REV;
    if (ticks < 0) {
        ticks = 0;
    }
    if (ticks > SERVO_TICKS_PER_REV - 1) {
        ticks = SERVO_TICKS_PER_REV - 1;
    }
    return (uint16_t)ticks;
}

static double ticks_to_radians(uint16_t ticks) {
    return ((double)ticks - CENTRE_TICK) / SERVO_TICKS_PER_REV * TWO_PI;
}

uint8_t servos_begin(void) {
#if !SERVOS_ENABLED
    return 0;
#else
    // One bus for the whole arm, wrapped so USB and the lidar keep running
    // while the driver waits for a servo to answer.
    bus = serial_hook_init(&hook, link101_servo_bus_init(&port, SERVO_BAUD), io_poll);
    if (!bus) {
        status_printf("[servo] no PIO state machine free; arm disabled\n");
        return 0;
    }

    uint8_t found = 0;
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        const servo_cfg_t *s = &SERVOS[i];

        online[i] = st3215_ping(bus, s->id);
        status_printf("[servo] id %u (%s): %s\n", s->id, s->joint,
                      online[i] ? "online" : "no response");
        if (!online[i]) {
            continue;
        }

        st3215_set_mode(bus, s->id, ST3215_MODE_POSITION);
        st3215_set_acceleration(bus, s->id, SERVO_DEFAULT_ACCEL);

        // Take torque with the servo already where it is, so switching on
        // doesn't yank the arm to some remembered position.
        int32_t here = st3215_read_position(bus, s->id);
        st3215_set_torque(bus, s->id, true);
        if (here >= 0) {
            st3215_move_to(bus, s->id, (uint16_t)here, 100, SERVO_DEFAULT_ACCEL);
            last_command[i] = ticks_to_radians((uint16_t)here) * s->direction;
        }

        sleep_ms(10);
        found++;
    }
    return found;
#endif
}

bool servos_online(uint8_t index) {
    return index < SERVO_COUNT && online[index];
}

void servos_set_angle(uint8_t index, double radians) {
    if (!servos_online(index)) {
        return;
    }
    if (fabs(radians - last_command[index]) < 1e-6) {
        return;   // same as last time; leave the bus alone
    }
    last_command[index] = radians;

    uint16_t speed = (uint16_t)(SERVO_MOVE_SPEED * SERVO_SPEED_SCALE);
    uint8_t  accel = (uint8_t)(SERVO_MOVE_ACCEL * SERVO_SPEED_SCALE);

    st3215_move_to(bus, SERVOS[index].id,
                   radians_to_ticks(radians * SERVOS[index].direction),
                   speed, accel);
}

bool servos_read_state(uint8_t index, double *radians, double *rad_per_sec) {
    if (!servos_online(index)) {
        return false;
    }

    uint16_t ticks;
    int16_t  steps_per_sec;
    if (!st3215_read_state(bus, SERVOS[index].id, &ticks, &steps_per_sec)) {
        return false;
    }

    double steps_per_radian = (double)SERVO_TICKS_PER_REV / TWO_PI;
    if (radians) {
        *radians = ticks_to_radians(ticks) * SERVOS[index].direction;
    }
    if (rad_per_sec) {
        *rad_per_sec = (steps_per_sec / steps_per_radian) * SERVOS[index].direction;
    }
    return true;
}

bool servos_read_telemetry(uint8_t index, st3215_telemetry_t *out) {
    if (!servos_online(index)) {
        return false;
    }
    return st3215_read_telemetry(bus, SERVOS[index].id, out);
}
