#!/usr/bin/env python3
"""
spray_visualizer – Manages Gazebo visual effects for the pesticide spray.

Crop Coloring (Object Pool Approach):
Maintains a small pool of 'blue_crop_cover' models hidden underground.
When the drone sprays a crop, a cover is teleported to the crop's position
for 1 second, then hidden again. This provides a temporary blue "glow" indicator
without any FPS drops from continuous Gazebo spawning.

Also publishes RViz POINTS markers on /spray/markers for RViz users.
"""

import math
import os
import random
from collections import deque

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from gazebo_msgs.srv import GetEntityState, SetEntityState, SpawnEntity
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist


class SprayVisualizer(Node):

    def __init__(self):
        super().__init__('spray_visualizer')

        self.declare_parameter('model_name', 'iris_sprayer')
        self.model_name = self.get_parameter('model_name').value

        # ── Load blue crop cover SDF ──
        pkg_share = get_package_share_directory('drone_sprayer')
        cover_sdf = os.path.join(pkg_share, 'models', 'blue_crop_cover', 'model.sdf')
        try:
            with open(cover_sdf, 'r') as f:
                self.cover_xml = f.read()
        except FileNotFoundError:
            self.cover_xml = None
            self.get_logger().error(f'blue_crop_cover SDF not found: {cover_sdf}')

        # ── State ──
        self.spray_active = False
        self.marker_id = 0

        # ── Drone position ──
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0

        # ── Crop grid: 10 rows × 20 columns ──
        self.crop_positions = {}
        for r in range(1, 11):
            x = 3.0 + (r - 1) * 2.0
            for c in range(1, 21):
                y = -9.5 + (c - 1) * 1.0
                self.crop_positions[(r, c)] = (x, y)

        self.spray_radius = 1.0

        # ── Object Pool for Crop Glows ──
        self.POOL_SIZE = 8
        self.glow_names_to_spawn = deque([f'crop_glow_{i}' for i in range(self.POOL_SIZE)])
        self.available_glows = deque()
        self.active_glows = {}   # mapping: glow_name -> expiration_time (float)
        self.crop_cooldowns = {} # mapping: (r, c) -> expiration_time (float)

        # ── Subscribers ──
        self.create_subscription(Bool, '/spray/active', self._on_spray, 10)
        self.create_subscription(String, '/drone/state', self._on_state, 10)

        # ── Publisher ──
        self.marker_pub = self.create_publisher(Marker, '/spray/markers', 10)

        # ── Gazebo service clients ──
        self.get_state_cli = self.create_client(
            GetEntityState, '/get_entity_state'
        )
        self.set_state_cli = self.create_client(
            SetEntityState, '/set_entity_state'
        )
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')

        # ── Timers ──
        self.timer = self.create_timer(0.1, self._tick)                 # 10 Hz
        self.pool_spawn_timer = self.create_timer(0.5, self._spawn_pool_tick) # Spawns pool slowly at startup

        self.get_logger().info('SprayVisualizer ready (Object Pool enabled).')

    def _on_spray(self, msg: Bool):
        self.spray_active = msg.data

    def _on_state(self, msg: String):
        pass

    # ──────────────────────────────────────────────
    #  Spawn the Object Pool Models at startup
    # ──────────────────────────────────────────────
    def _spawn_pool_tick(self):
        if not self.glow_names_to_spawn:
            self.pool_spawn_timer.cancel()
            self.get_logger().info('All crop glow markers spawned and ready.')
            return
            
        if self.cover_xml is None or not self.spawn_cli.service_is_ready():
            return

        name = self.glow_names_to_spawn.popleft()
        req = SpawnEntity.Request()
        req.name = name
        req.xml = self.cover_xml
        req.robot_namespace = ''
        req.reference_frame = 'world'
        req.initial_pose = Pose()
        req.initial_pose.position.x = 0.0
        req.initial_pose.position.y = 0.0
        req.initial_pose.position.z = -50.0 # Hide underground
        
        future = self.spawn_cli.call_async(req)
        future.add_done_callback(lambda f, n=name: self._on_pool_spawn_done(f, n))

    def _on_pool_spawn_done(self, future, name):
        try:
            resp = future.result()
            if resp.success:
                self.available_glows.append(name)
        except Exception as e:
            self.get_logger().debug(f'Spawn pool error for {name}: {e}')

    # ──────────────────────────────────────────────
    #  Main 10 Hz tick
    # ──────────────────────────────────────────────
    def _tick(self):
        # Time in seconds as float
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Cleanup expired glows
        glows_to_hide = []
        for name, exp_time in list(self.active_glows.items()):
            if current_time >= exp_time:
                glows_to_hide.append(name)
                
        for name in glows_to_hide:
            self._teleport_entity(name, 0.0, 0.0, -50.0)
            del self.active_glows[name]
            self.available_glows.append(name)

        if not self.spray_active:
            # Clear RViz markers
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'spray'
            m.id = 0
            m.action = Marker.DELETEALL
            self.marker_pub.publish(m)
            return

        # Poll drone position for crop hit detection
        if not self.get_state_cli.service_is_ready():
            return
        req = GetEntityState.Request()
        req.name = self.model_name
        req.reference_frame = 'world'
        future = self.get_state_cli.call_async(req)
        future.add_done_callback(self._on_pose)

    def _on_pose(self, future):
        try:
            resp = future.result()
        except Exception:
            return
        if not resp.success:
            return

        self.drone_x = resp.state.pose.position.x
        self.drone_y = resp.state.pose.position.y
        self.drone_z = resp.state.pose.position.z

        # RViz particles (water droplet effect)
        self._publish_rviz_particles(self.drone_x, self.drone_y, self.drone_z)

        # Trigger temporary blue glow on hit crops
        self._check_crop_hits(self.drone_x, self.drone_y)

    # ──────────────────────────────────────────────
    #  RViz markers (water droplets)
    # ──────────────────────────────────────────────
    def _publish_rviz_particles(self, px, py, pz):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'spray'
        marker.id = self.marker_id
        self.marker_id = (self.marker_id + 1) % 10000
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.color.r = 0.0
        marker.color.g = 0.8
        marker.color.b = 0.4
        marker.color.a = 0.7
        marker.lifetime.sec = 1

        for _ in range(30):
            drop_z = random.uniform(0.0, pz - 0.2)
            spread = (pz - drop_z) * 0.25
            pt = Point()
            pt.x = px + random.gauss(0, spread)
            pt.y = py + random.gauss(0, spread)
            pt.z = drop_z
            marker.points.append(pt)

        self.marker_pub.publish(marker)

    # ──────────────────────────────────────────────
    #  Crop Hit Logic - uses Object Pool teleportation
    # ──────────────────────────────────────────────
    def _check_crop_hits(self, dx, dy):
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        for key, (cx, cy) in self.crop_positions.items():
            # Check cooldown to avoid re-triggering while drone still hovers
            if key in self.crop_cooldowns and current_time < self.crop_cooldowns[key]:
                continue
                
            dist = math.sqrt((dx - cx) ** 2 + (dy - cy) ** 2)
            if dist <= self.spray_radius:
                if self.available_glows:
                    glow_name = self.available_glows.popleft()
                    self._teleport_entity(glow_name, cx, cy, 0.0)
                    self.active_glows[glow_name] = current_time + 1.0 # Show for 1 sec
                    self.crop_cooldowns[key] = current_time + 5.0     # 5 sec cooldown prevent re-spray
                else:
                    # No available glows right now
                    pass

    # ──────────────────────────────────────────────
    #  Teleport Entity
    # ──────────────────────────────────────────────
    def _teleport_entity(self, name, x, y, z):
        if not self.set_state_cli.service_is_ready():
            return
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = name
        req.state.pose = Pose()
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = float(z)
        req.state.twist = Twist()
        req.state.reference_frame = 'world'
        self.set_state_cli.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = SprayVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
