#include "imu.h"

#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "link101/lsm6dsox.h"
#include "link101/mmc5983.h"

#include "robot.h"
#include "status.h"

static link101_lsm6dsox_t sensor;
static link101_mmc5983_t  magnetometer;
static bool               sensor_ready;
static bool               mag_ready;

bool imu_begin(void) {
    i2c_init(IMU_I2C, IMU_I2C_BAUD);
    gpio_set_function(LINK101_PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(LINK101_PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(LINK101_PIN_SDA);
    gpio_pull_up(LINK101_PIN_SCL);

    sensor_ready = link101_lsm6dsox_init(&sensor, IMU_I2C, IMU_ADDR);
    if (!sensor_ready) {
        // SDO/SA0 picks the low address bit. If the board strap doesn't
        // match what robot.h assumes -- a revision difference, or a strap
        // this firmware guessed wrong -- the chip answers at the other
        // address instead of not answering at all. Try it before giving up.
        sensor_ready = link101_lsm6dsox_init(&sensor, IMU_I2C, LINK101_LSM6DSOX_ADDR_ALT);
        if (sensor_ready) {
            status_printf("[imu  ] LSM6DSOX answered at 0x%02X, not the expected 0x%02X "
                          "-- SDO/SA0 strap differs from robot.h\n",
                          LINK101_LSM6DSOX_ADDR_ALT, IMU_ADDR);
        }
    }
    status_printf("[imu  ] LSM6DSOX: %s\n",
                  sensor_ready ? "online" : "no response at 0x6B or 0x6A");

    if (!sensor_ready) {
        // Both addresses failed init(), which only ever says "no" -- not
        // *why*. This says why: whether nothing is on the bus at either
        // address at all, or something is, just not what we expected.
        // Whichever it is, the next line pins it down instead of guessing.
        uint8_t id;
        if (link101_lsm6dsox_probe(IMU_I2C, IMU_ADDR, &id)) {
            status_printf("[imu  ]   0x%02X: answered, WHO_AM_I=0x%02X (expected 0x%02X)\n",
                          IMU_ADDR, id, LINK101_LSM6DSOX_WHO_AM_I);
        } else {
            status_printf("[imu  ]   0x%02X: no ack at all\n", IMU_ADDR);
        }
        if (link101_lsm6dsox_probe(IMU_I2C, LINK101_LSM6DSOX_ADDR_ALT, &id)) {
            status_printf("[imu  ]   0x%02X: answered, WHO_AM_I=0x%02X (expected 0x%02X)\n",
                          LINK101_LSM6DSOX_ADDR_ALT, id, LINK101_LSM6DSOX_WHO_AM_I);
        } else {
            status_printf("[imu  ]   0x%02X: no ack at all\n", LINK101_LSM6DSOX_ADDR_ALT);
        }
    }

    mag_ready = link101_mmc5983_init(&magnetometer, IMU_I2C, MAG_ADDR);
    status_printf("[imu  ] MMC5983MA at 0x%02X: %s\n", MAG_ADDR,
                  mag_ready ? "online" : "no response");

    return sensor_ready || mag_ready;
}

bool imu_online(void) {
    return sensor_ready;
}

bool imu_mag_online(void) {
    return mag_ready;
}

bool imu_read(imu_sample_t *out) {
    if (!sensor_ready) {
        return false;
    }
    if (!link101_lsm6dsox_read(&sensor, out->accel, out->gyro)) {
        return false;
    }
    // Temperature is a separate register block and nobody minds if it is a
    // sample behind, so a failure here doesn't spoil the reading.
    if (!link101_lsm6dsox_read_temperature(&sensor, &out->temp_c)) {
        out->temp_c = 0.0f;
    }
    return true;
}

bool imu_read_magnetic_field(float tesla[3]) {
    return mag_ready && link101_mmc5983_read(&magnetometer, tesla);
}
