#!/usr/bin/env python3
"""
drone_controller – Main flight-control node for the IRIS pesticide-spraying drone.

State machine:
  IDLE → TAKEOFF → NAVIGATE → SPRAYING → RETURN → LAND → IDLE

Uses gazebo_msgs/srv/SetEntityState to move the drone (position control).
"""

import math
import random
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, Twist, Point
from gazebo_msgs.srv import SetEntityState


class DroneController(Node):

    # ── Field layout constants (match crop_field.world) ──
    FIELD_X_START = 3.0      # first crop row X
    FIELD_X_END = 21.0       # last crop row X
    FIELD_Y_START = -9.5     # first column Y
    FIELD_Y_END = 9.5        # last column Y
    ROW_SPACING = 2.0        # X spacing between rows
    BASE_X = 0.0
    BASE_Y = 0.0
    BASE_Z = 0.195           # ground rest altitude

    def __init__(self):
        super().__init__('drone_controller')

        # ── Parameters ──
        self.declare_parameter(
            'spray_altitude', 3.0,
            ParameterDescriptor(dynamic_typing=True)
        )
        self.declare_parameter(
            'drone_speed', 2.0,
            ParameterDescriptor(dynamic_typing=True)
        )
        self.declare_parameter('model_name', 'iris_sprayer')

        self.altitude = float(self.get_parameter('spray_altitude').value)
        self.speed = float(self.get_parameter('drone_speed').value)
        self.model_name = self.get_parameter('model_name').value

        # ── State ──
        self.state = 'IDLE'
        self.current_pose = [self.BASE_X, self.BASE_Y, self.BASE_Z]
        self.waypoints = []
        self.wp_index = 0

        # ── Publishers ──
        self.state_pub = self.create_publisher(String, '/drone/state', 10)
        self.spray_active_pub = self.create_publisher(Bool, '/spray/active', 10)

        # ── Subscribers ──
        self.create_subscription(Bool, '/spray/start', self._on_spray_start, 10)

        # ── Gazebo service client ──
        self.set_state_cli = self.create_client(SetEntityState, '/set_entity_state')
        self.get_logger().info('Waiting for /set_entity_state service ...')
        self.set_state_cli.wait_for_service(timeout_sec=30.0)
        self.get_logger().info('Service available.')

        # ── Control loop timer (20 Hz) ──
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self._control_loop)

        self._publish_state()
        self.get_logger().info(
            f'DroneController ready — altitude={self.altitude}, speed={self.speed}'
        )

    # ────────────────────────────────────────────────────────────
    # Callback: /spray/start
    # ────────────────────────────────────────────────────────────
    def _on_spray_start(self, msg: Bool):
        if msg.data and self.state == 'IDLE':
            self.get_logger().info('🚀 Spray mission triggered!')
            self._begin_mission()

    # ────────────────────────────────────────────────────────────
    # Mission planner
    # ────────────────────────────────────────────────────────────
    def _begin_mission(self):
        """Build the waypoint list for the entire spray mission."""
        alt = self.altitude
        wps = []

        # 1. Takeoff
        wps.append(('TAKEOFF', [self.BASE_X, self.BASE_Y, alt]))

        # Randomly choose a start corner
        start_x = random.choice([self.FIELD_X_START, self.FIELD_X_END])
        start_y = random.choice([self.FIELD_Y_START, self.FIELD_Y_END])

        # Generate X coordinates depending on starting side
        rows_x = []
        if start_x == self.FIELD_X_START:
            # Sweep Left to Right
            x = self.FIELD_X_START
            while x <= self.FIELD_X_END + 0.1:
                rows_x.append(x)
                x += self.ROW_SPACING
        else:
            # Sweep Right to Left
            x = self.FIELD_X_END
            while x >= self.FIELD_X_START - 0.1:
                rows_x.append(x)
                x -= self.ROW_SPACING

        current_y = start_y

        # 2. Fly to chosen field entry corner
        wps.append(('NAVIGATE', [rows_x[0], current_y, alt]))

        # 3. Spray each row
        for rx in rows_x:
            # The other end of the current row
            next_y = self.FIELD_Y_END if current_y == self.FIELD_Y_START else self.FIELD_Y_START
            
            # Spray along the row length
            wps.append(('SPRAYING', [rx, current_y, alt]))
            wps.append(('SPRAYING', [rx, next_y, alt]))
            
            # Update position for the next iteration (move horizontally between rows)
            current_y = next_y

        # 4. Return to base at altitude
        wps.append(('RETURN', [self.BASE_X, self.BASE_Y, alt]))

        # 5. Land
        wps.append(('LAND', [self.BASE_X, self.BASE_Y, self.BASE_Z]))

        self.waypoints = wps
        self.wp_index = 0
        self.state = 'TAKEOFF'
        self._publish_state()
        self.get_logger().info(f'Mission planned with {len(wps)} waypoints starting from corner X={start_x}, Y={start_y}.')

    # ────────────────────────────────────────────────────────────
    # Control loop (20 Hz)
    # ────────────────────────────────────────────────────────────
    def _control_loop(self):
        if self.state == 'IDLE':
            return

        if self.wp_index >= len(self.waypoints):
            # Mission complete
            self.state = 'IDLE'
            self._publish_state()
            self._publish_spray(False)
            self.get_logger().info('✅ Mission complete — drone is IDLE.')
            return

        label, target = self.waypoints[self.wp_index]

        # Update state label if changed
        if self.state != label:
            self.state = label
            self._publish_state()

        # Spray active while in SPRAYING state
        self._publish_spray(self.state == 'SPRAYING')

        # Move towards target
        reached = self._move_towards(target)

        if reached:
            self.get_logger().info(
                f'  ✔ WP {self.wp_index}: {label}  → '
                f'({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f})'
            )
            self.wp_index += 1

    # ────────────────────────────────────────────────────────────
    # Movement helper — returns True when target reached
    # ────────────────────────────────────────────────────────────
    def _move_towards(self, target) -> bool:
        dx = target[0] - self.current_pose[0]
        dy = target[1] - self.current_pose[1]
        dz = target[2] - self.current_pose[2]
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        step = self.speed * self.dt  # distance per tick

        if dist <= step:
            # Snap to target
            self.current_pose = list(target)
            self._set_gazebo_pose(self.current_pose)
            return True

        # Normalise and step
        ratio = step / dist
        self.current_pose[0] += dx * ratio
        self.current_pose[1] += dy * ratio
        self.current_pose[2] += dz * ratio
        self._set_gazebo_pose(self.current_pose)
        return False

    # ────────────────────────────────────────────────────────────
    # Gazebo state setter (async, fire-and-forget)
    # ────────────────────────────────────────────────────────────
    def _set_gazebo_pose(self, pos):
        req = SetEntityState.Request()
        req.state.name = self.model_name
        req.state.pose = Pose()
        req.state.pose.position = Point(x=pos[0], y=pos[1], z=pos[2])
        req.state.pose.orientation.w = 1.0
        req.state.twist = Twist()
        req.state.reference_frame = 'world'
        future = self.set_state_cli.call_async(req)
        future.add_done_callback(self._on_set_state_response)

    def _on_set_state_response(self, future):
        try:
            resp = future.result()
            if not resp.success:
                self.get_logger().warn(
                    f'SetEntityState FAILED: {resp.status_message}'
                )
        except Exception as e:
            self.get_logger().error(f'SetEntityState call error: {e}')

    # ────────────────────────────────────────────────────────────
    # Publishers
    # ────────────────────────────────────────────────────────────
    def _publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)
        self.get_logger().info(f'State → {self.state}')

    def _publish_spray(self, active: bool):
        msg = Bool()
        msg.data = active
        self.spray_active_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
