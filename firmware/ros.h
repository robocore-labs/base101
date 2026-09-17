/*
 * The ROS 2 side of the robot.
 *
 * The board is a ROS node in its own right: it subscribes to the command
 * topics and publishes joint states, servo telemetry and IMU data, over
 * zenoh on USB CDC #0. On the host, rmw_zenoh makes all of that look like
 * any other node -- there is no bridge process and no serial protocol to
 * speak.
 *
 * Topic names, rates and frame ids are in robot.h.
 */

#ifndef ROS_H
#define ROS_H

#include <stdbool.h>

// Connect to the zenoh router, declare the node, and create every publisher
// and subscriber. Blocks until the router shows up -- USB and the lidar
// keep running while it waits. Returns false only if the node itself could
// not be declared.
bool ros_begin(void);

// Receive whatever arrived (command callbacks fire from in here), then
// publish whatever is due. Call it every time round the main loop.
void ros_update(void);

#endif // ROS_H
