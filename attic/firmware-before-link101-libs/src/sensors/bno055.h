/*
 * Bosch BNO055 9-DOF IMU driver (I2C).
 *
 * Configured for NDOF fusion mode, which provides an absolute-orientation
 * quaternion in addition to the raw accelerometer, gyroscope and
 * magnetometer data. Units are converted to ROS conventions on read:
 *   - quaternion   unit (w, x, y, z)
 *   - gyro         rad/s
 *   - accel        m/s^2 (includes gravity, as sensor_msgs/Imu expects)
 *   - mag          tesla
 *   - temperature  deg C
 */

#ifndef SENSORS_BNO055_H
#define SENSORS_BNO055_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "hardware/i2c.h"

#define BNO055_ADDR_DEFAULT 0x28
#define BNO055_ADDR_ALT     0x29

typedef struct {
    float quat[4];   // unit quaternion, order [w, x, y, z]
    float gyro[3];   // angular velocity, rad/s
    float accel[3];  // linear acceleration incl. gravity, m/s^2
    float mag[3];    // magnetic field, tesla
    int8_t temp_c;   // chip temperature, deg C
} bno055_sample_t;

// Verify the chip id, configure units and enter NDOF fusion mode. The I2C
// bus must already be initialized. Returns true once the sensor answers with
// the expected chip id (it is retried for a few hundred ms to cover the
// BNO055's slow power-on boot).
bool bno055_init(i2c_inst_t *i2c, uint8_t addr);

// Read a full sample (fusion quaternion + gyro + accel + mag + temp).
// Returns false on any I2C error.
bool bno055_read(bno055_sample_t *out);

#endif // SENSORS_BNO055_H
