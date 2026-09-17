/*
 * base101 firmware — a ROS 2 node on a Link101 board (RP2350A).
 * ============================================================
 *
 * The robot's motor control runs here, on the board, and the host talks to
 * it in ROS: it subscribes to command topics and publishes joint states,
 * servo telemetry and IMU data over zenoh. There is no bridge process on
 * the host and no serial protocol to speak -- with rmw_zenoh running, this
 * board is just another node in the graph.
 *
 *   USB port     What comes out of it
 *   ----------------------------------------------------------------
 *   CDC #0       zenoh: the ROS traffic
 *   CDC #1       the lidar, passed straight through
 *   CDC #2       this firmware's boot log, until zenoh comes up
 *
 *   Bus          What is on it
 *   ----------------------------------------------------------------
 *   4x PIO UART  one DDSM210 wheel motor each     (wheels.c)
 *   servo bus    the Feetech arm, half duplex     (servos.c)
 *   uart1        RPLidar C1                       (lidar.c)
 *   i2c1         onboard LSM6DSOX + MMC5983MA IMU (imu.c)
 *
 * Everything you would want to change -- pins, motor IDs, joint names,
 * rates, topic names -- is in robot.h. Start there.
 *
 * Read the rest in this order: main.c for the shape, then robot.h for the
 * robot, then whichever of wheels.c / servos.c / imu.c / lidar.c / ros.c
 * you care about. Each one is about a hundred lines and stands alone.
 */

#include "pico/stdlib.h"

#include "imu.h"
#include "io.h"
#include "lidar.h"
#include "robot.h"
#include "ros.h"
#include "servos.h"
#include "status.h"
#include "wheels.h"

// Bring the robot up, narrating to the debug port as we go. Anything that
// doesn't answer is reported and skipped -- a missing servo or an
// unplugged lidar is not a reason to refuse to boot.
static void setup(void) {
    io_begin();        // USB first, so the rest of this is readable on CDC #2
    status_begin();

    status_printf("\n\n=== base101 firmware ===\n");
    status_printf("[boot ] USB up: CDC0 zenoh, CDC1 lidar, CDC2 this log\n");

    lidar_begin();
    status_printf("[lidar] uart1 on GP%u/%u at %u baud\n",
                  LIDAR_TX_PIN, LIDAR_RX_PIN, LIDAR_BAUD);

    uint8_t found = wheels_begin();
    status_printf("[wheel] %u of %u wheels answered\n", found, WHEEL_COUNT);
    // Start stopped, actively -- not in whatever state the motors happened
    // to power up in. Matters most for a robot that lost power mid-drive:
    // without this, a wheel could resume its last remembered speed the
    // instant wheels_begin() puts it back in velocity mode.
    for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
        wheels_brake(i);
    }

#if SERVOS_ENABLED
    found = servos_begin();
    status_printf("[servo] %u of %u servos answered\n", found, SERVO_COUNT);
#else
    status_printf("[servo] arm switched off in robot.h\n");
#endif

    imu_begin();

    // Blocks until the router is up -- the strip blinks yellow throughout,
    // which is the only signal you get once the boot log has scrolled past.
    // USB and the lidar keep running while it waits, so the board stays
    // usable however long that takes.
    if (!ros_begin()) {
        status_printf("[ros  ] FATAL: could not declare the node\n");
        while (true) {
            io_poll();
            status_update();
        }
    }

    // Connected: green from here, and slow.
    status_set_mode(STATUS_READY);

    // From here on the debug port would be competing with zenoh for USB
    // bandwidth, and zenoh wins. The LED keeps breathing; that is how you
    // know the loop below is still turning.
    status_printf("[boot ] up. Going quiet -- watch the LED, or the ROS graph.\n");
    status_quiet();
}

// The whole steady state: receive commands, publish state, keep USB and
// the lidar moving, breathe.
static void loop(void) {
    ros_update();
    wheels_update();   // advance the speed ramp; rate-limits itself
    io_poll();
    status_update();
}

int main(void) {
    stdio_init_all();

    setup();
    while (true) {
        loop();
    }
}
