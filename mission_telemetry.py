#!/usr/bin/env python3
"""
mission_telemetry – Aggregator node for all drone spray mission telemetry.

Subscribes to raw topics from drone_controller and spray_visualizer, computes
derived metrics (battery, coverage, efficiency, overlap), and publishes a
unified JSON telemetry message on /drone/telemetry at 2 Hz.

Computed parameters:
  1.  Drone Position (X, Y)
  2.  Flight Path pattern
  3.  Time Taken (mission wall-clock)
  4.  Spray Coverage Area (%)
  5.  Spray Status ON / OFF
  6.  Battery Percentage (estimated)
  7.  Distance Covered (m)
  8.  Speed (m/s)
  9.  Payload Weight (kg)
  10. Spray Efficiency (%)
  11. Overlap / Wastage (%)
  12. Field Size
"""

import json
import math
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String, Bool, Float64
from geometry_msgs.msg import PointStamped


class MissionTelemetry(Node):

    def __init__(self):
        super().__init__('mission_telemetry')

        # ── Parameters ──
        self.declare_parameter(
            'battery_capacity_pct', 100.0,
            ParameterDescriptor(description='Starting battery percentage')
        )
        self.declare_parameter(
            'base_drain_per_sec', 0.15,
            ParameterDescriptor(description='Battery drain per second (flight)')
        )
        self.declare_parameter(
            'spray_drain_per_sec', 0.25,
            ParameterDescriptor(description='Extra battery drain per second while spraying')
        )
        self.declare_parameter(
            'weight_drain_factor', 0.02,
            ParameterDescriptor(description='Extra battery drain per kg of payload per second')
        )

        self.battery_pct = float(self.get_parameter('battery_capacity_pct').value)
        self.base_drain = float(self.get_parameter('base_drain_per_sec').value)
        self.spray_drain = float(self.get_parameter('spray_drain_per_sec').value)
        self.weight_factor = float(self.get_parameter('weight_drain_factor').value)

        # ── State from subscriptions ──
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.drone_state = 'IDLE'
        self.spray_active = False
        self.distance_covered = 0.0
        self.speed = 0.0
        self.payload_weight = 10.0
        self.flight_path = 'GRID'
        self.field_info = {
            'field_size': 'medium',
            'x_start': 3.0, 'x_end': 21.0,
            'y_start': -9.5, 'y_end': 9.5,
            'total_area_m2': 342.0,
        }

        # ── Mission timing ──
        self.mission_active = False
        self.mission_start_time = None
        self.mission_elapsed_sec = 0.0

        # ── Coverage tracking ──
        # Spatial hash: 1 m² cells  →  set of (cell_x, cell_y)
        self.sprayed_cells = set()
        self.total_spray_events = 0        # every cell-spray event (including repeats)
        self.cell_size = 1.0               # metres per grid cell

        # ── Subscribers ──
        self.create_subscription(PointStamped, '/drone/position', self._on_position, 10)
        self.create_subscription(String, '/drone/state', self._on_state, 10)
        self.create_subscription(Bool, '/spray/active', self._on_spray, 10)
        self.create_subscription(Float64, '/drone/distance_covered', self._on_distance, 10)
        self.create_subscription(Float64, '/drone/speed', self._on_speed, 10)
        self.create_subscription(Float64, '/drone/payload_weight', self._on_payload, 10)
        self.create_subscription(String, '/drone/flight_path', self._on_flight_path, 10)
        self.create_subscription(String, '/drone/field_info', self._on_field_info, 10)

        # ── Publisher ──
        self.telemetry_pub = self.create_publisher(String, '/drone/telemetry', 10)

        # ── Timer (2 Hz) ──
        self.telemetry_dt = 0.5
        self.create_timer(self.telemetry_dt, self._publish_telemetry)

        self.get_logger().info('MissionTelemetry aggregator ready.')

    # ────────────────────────────────────────────────────────────
    # Subscription callbacks
    # ────────────────────────────────────────────────────────────
    def _on_position(self, msg: PointStamped):
        self.position = {'x': msg.point.x, 'y': msg.point.y, 'z': msg.point.z}

        # Track spray coverage when spraying
        if self.spray_active:
            cx = int(math.floor(msg.point.x / self.cell_size))
            cy = int(math.floor(msg.point.y / self.cell_size))
            cell = (cx, cy)
            if cell in self.sprayed_cells:
                # This is an overlap / re-spray event
                pass
            self.sprayed_cells.add(cell)
            self.total_spray_events += 1

    def _on_state(self, msg: String):
        prev = self.drone_state
        self.drone_state = msg.data

        # Mission timing logic
        if prev == 'IDLE' and self.drone_state != 'IDLE':
            # Mission just started
            self.mission_active = True
            self.mission_start_time = self.get_clock().now().nanoseconds / 1e9
            self.mission_elapsed_sec = 0.0
            # Reset coverage for new mission
            self.sprayed_cells.clear()
            self.total_spray_events = 0
            # Reset battery
            self.battery_pct = float(self.get_parameter('battery_capacity_pct').value)
            self.get_logger().info('📊 Telemetry: Mission started — counters reset.')

        if prev != 'IDLE' and self.drone_state == 'IDLE':
            # Mission just ended
            self.mission_active = False
            self.get_logger().info(
                f'📊 Telemetry: Mission ended — elapsed={self.mission_elapsed_sec:.1f}s, '
                f'coverage={self._compute_coverage():.1f}%'
            )

    def _on_spray(self, msg: Bool):
        self.spray_active = msg.data

    def _on_distance(self, msg: Float64):
        self.distance_covered = msg.data

    def _on_speed(self, msg: Float64):
        self.speed = msg.data

    def _on_payload(self, msg: Float64):
        self.payload_weight = msg.data

    def _on_flight_path(self, msg: String):
        self.flight_path = msg.data

    def _on_field_info(self, msg: String):
        try:
            self.field_info = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    # ────────────────────────────────────────────────────────────
    # Derived metrics
    # ────────────────────────────────────────────────────────────
    def _compute_coverage(self) -> float:
        """Spray coverage: unique cells sprayed / total field cells × 100."""
        total_area = self.field_info.get('total_area_m2', 342.0)
        total_cells = total_area / (self.cell_size ** 2)
        if total_cells <= 0:
            return 0.0
        return min(100.0, (len(self.sprayed_cells) / total_cells) * 100.0)

    def _compute_efficiency(self) -> float:
        """Spray efficiency: unique cells / total spray events × 100."""
        if self.total_spray_events <= 0:
            return 0.0
        return min(100.0, (len(self.sprayed_cells) / self.total_spray_events) * 100.0)

    def _compute_overlap(self) -> float:
        """Overlap / wastage: repeated spray events as % of total events."""
        if self.total_spray_events <= 0:
            return 0.0
        repeats = self.total_spray_events - len(self.sprayed_cells)
        return max(0.0, (repeats / self.total_spray_events) * 100.0)

    def _drain_battery(self):
        """Simulate battery drain per tick."""
        if not self.mission_active:
            return
        drain = self.base_drain * self.telemetry_dt
        if self.spray_active:
            drain += self.spray_drain * self.telemetry_dt
        drain += self.weight_factor * self.payload_weight * self.telemetry_dt
        self.battery_pct = max(0.0, self.battery_pct - drain)

    # ────────────────────────────────────────────────────────────
    # Publish aggregated telemetry (2 Hz)
    # ────────────────────────────────────────────────────────────
    def _publish_telemetry(self):
        # Update mission elapsed time
        if self.mission_active and self.mission_start_time is not None:
            now = self.get_clock().now().nanoseconds / 1e9
            self.mission_elapsed_sec = now - self.mission_start_time

        # Simulate battery drain
        self._drain_battery()

        telemetry = {
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            # 1. Drone Position
            'position': self.position,
            # 2. Flight Path
            'flight_path': self.flight_path,
            # 3. Time Taken
            'time_taken_sec': round(self.mission_elapsed_sec, 2),
            # 4. Spray Coverage Area (%)
            'spray_coverage_pct': round(self._compute_coverage(), 2),
            # 5. Spray Status
            'spray_active': self.spray_active,
            # 6. Battery Percentage
            'battery_pct': round(self.battery_pct, 2),
            # 7. Distance Covered
            'distance_covered_m': round(self.distance_covered, 2),
            # 8. Speed
            'speed_mps': round(self.speed, 2),
            # 9. Payload Weight
            'payload_weight_kg': round(self.payload_weight, 2),
            # 10. Spray Efficiency
            'spray_efficiency_pct': round(self._compute_efficiency(), 2),
            # 11. Overlap / Wastage
            'overlap_wastage_pct': round(self._compute_overlap(), 2),
            # 12. Field Size
            'field_size': self.field_info.get('field_size', 'medium'),
            'field_dimensions': {
                'x_start': self.field_info.get('x_start', 3.0),
                'x_end': self.field_info.get('x_end', 21.0),
                'y_start': self.field_info.get('y_start', -9.5),
                'y_end': self.field_info.get('y_end', 9.5),
            },
            # Mission state
            'state': self.drone_state,
            'mission_active': self.mission_active,
        }

        msg = String()
        msg.data = json.dumps(telemetry)
        self.telemetry_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionTelemetry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
