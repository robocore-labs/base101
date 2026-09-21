#!/usr/bin/env python3
"""Responds to the Axon 2 firmware's time-sync probes.

The firmware sends its monotonic transmit time (microseconds) on
/link101/time_sync/request at 1 Hz and estimates the host<->board clock
offset from the round trip, using the two host timestamps this node adds. It
rejects exchanges over 20 ms total, so the callback below does the minimum:
read the clock, copy the payload, read the clock again, publish. No timer —
purely reactive, one response per request.
"""

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Int64MultiArray, UInt64

INT64_MAX = 2 ** 63 - 1

_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class Base101Time(Node):
    def __init__(self):
        super().__init__('base101_time')
        # Explicit SYSTEM_TIME clock: the offset estimate the firmware builds
        # from these timestamps has to be against the real wall clock, so
        # this must stay correct even if use_sim_time:=true leaks in from the
        # rest of the launch tree or /clock is being published.
        self._clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self._pub = self.create_publisher(
            Int64MultiArray, '/link101/time_sync/response', _QOS)
        self.create_subscription(
            UInt64, '/link101/time_sync/request', self._on_request, _QOS)

    def _on_request(self, msg):
        host_receive_ns = self._clock.now().nanoseconds
        board_transmit_us = msg.data

        # The response carries signed int64s; a request that can't round-trip
        # unchanged would corrupt the offset estimate, so drop it instead.
        if board_transmit_us > INT64_MAX:
            self.get_logger().warn(
                'time_sync request %d exceeds INT64_MAX; dropping'
                % board_transmit_us)
            return

        response = Int64MultiArray()
        response.layout.dim = []
        response.layout.data_offset = 0
        host_send_ns = self._clock.now().nanoseconds
        response.data = [int(board_transmit_us), host_receive_ns, host_send_ns]
        self._pub.publish(response)


def main():
    rclpy.init()
    node = Base101Time()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
