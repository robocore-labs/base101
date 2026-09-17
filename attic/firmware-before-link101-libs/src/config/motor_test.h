/*
 * Direct motor test helpers for config mode (bench testing over the JSON
 * console, bypassing ROS/zenoh). Only meaningful once the motor buses are up.
 */

#ifndef AXON_MOTOR_TEST_H
#define AXON_MOTOR_TEST_H

// Discover hardware on the buses (no ROS config used): sweep each DDSM bus for
// its motor and the ST bus for servos, printing what answers as JSON:
//   {"motors":{"wheels":[{"index":0,"port":"FL","online":true,"id":1,"mode":2},...],
//              "servos":[{"id":1},...],"servo_scan":[1,30]}}
void motor_test_list_json(void);

// Spin one DDSM wheel (by index 0..AXON_DDSM_COUNT-1) at the given RPM.
void motor_test_wheel(int index, double rpm);

// Move an ST3215 servo (by bus ID) to an absolute position (0..4095 ticks).
void motor_test_servo(int id, int pos);

// Move an ST3215 servo by a relative tick delta from its current position.
void motor_test_servo_delta(int id, int delta);

// Stop all wheels.
void motor_test_stop_all(void);

// Change an ST3215 servo's bus ID (old_id -> new_id). Best done with a single
// servo on the bus. Reports the result as JSON.
void motor_test_set_servo_id(int old_id, int new_id);

#endif // AXON_MOTOR_TEST_H
