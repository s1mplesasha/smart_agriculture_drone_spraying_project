#!/usr/bin/env python3
"""
spray_scheduler – Timer-based scheduler that triggers spray missions at regular intervals.

Publishes True to /spray/start every `spray_interval_sec` seconds.
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Bool, String


class SprayScheduler(Node):

    def __init__(self):
        super().__init__('spray_scheduler')

        # ── Parameters ──
        self.declare_parameter(
            'spray_interval_sec', 60.0,
            ParameterDescriptor(dynamic_typing=True)
        )
        self.interval = float(self.get_parameter('spray_interval_sec').value)

        # ── Publishers ──
        self.start_pub = self.create_publisher(Bool, '/spray/start', 10)
        self.info_pub = self.create_publisher(String, '/spray/schedule_info', 10)

        # ── State ──
        self.mission_count = 0
        self.first_trigger = True   # trigger immediately on first tick

        # ── Timer ──
        self.timer = self.create_timer(self.interval, self._on_timer)

        self.get_logger().info(
            f'SprayScheduler started — interval = {self.interval}s.  '
            f'First mission will be triggered immediately.'
        )

        # Trigger the first mission after a short delay (let Gazebo settle)
        self.create_timer(5.0, self._initial_trigger)

    def _initial_trigger(self):
        """Send the very first trigger after a 5-second start-up delay."""
        if self.first_trigger:
            self.first_trigger = False
            self._trigger_mission()

    def _on_timer(self):
        """Periodic timer callback."""
        self._trigger_mission()

    def _trigger_mission(self):
        self.mission_count += 1
        self.get_logger().info(
            f'⏰ Triggering spray mission #{self.mission_count}'
        )

        # Publish start command
        msg = Bool()
        msg.data = True
        self.start_pub.publish(msg)

        # Publish info
        info = String()
        info.data = (
            f'Mission #{self.mission_count} triggered.  '
            f'Next mission in {self.interval}s.'
        )
        self.info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = SprayScheduler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
