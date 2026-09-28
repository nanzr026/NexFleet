#!/usr/bin/env python3

import json
import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String, Empty

from .warehouse_map import WarehouseMap
from .astar_planner import AStarPlanner
from .pickup_geometry import choose_approach, docking_distance


class AMRController(Node):
    WORLD_FRAME = 'world_demo'
    LOCALIZATION_SOURCE = 'gazebo_ground_truth'
    MAP_CLASS = WarehouseMap


    # Gazebo/world spawn coordinates.
    ROBOT_SPAWNS = {
        'amr_01': (13.9, -10.6),
        'amr_02': (13.9, -8.0),
        'amr_03': (13.9, -5.4),
    }

    def __init__(self):
        super().__init__('amr_controller')

        # =====================================================
        # ROBOT CONFIGURATION
        # =====================================================

        self.declare_parameter('robot_id', 'amr_01')
        self.declare_parameter('initial_battery', 100.0)

        self.robot_id = (
            self.get_parameter('robot_id')
            .get_parameter_value()
            .string_value
        )

        self.battery_level = float(
            self.get_parameter('initial_battery').value
        )

        if self.robot_id not in self.ROBOT_SPAWNS:
            raise ValueError(
                f'Unknown robot_id: {self.robot_id}'
            )

        self.spawn_x, self.spawn_y = (
            self.ROBOT_SPAWNS[self.robot_id]
        )

        # =====================================================
        # LOCAL ODOMETRY
        # =====================================================

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.odom_received = False

        # =====================================================
        # TASK
        # =====================================================

        self.current_task_id = None

        self.pickup_world_x = None
        self.pickup_world_y = None

        self.delivery_world_x = None
        self.delivery_world_y = None

        self.approach_world_x = None
        self.approach_world_y = None

        self.target_world_x = None
        self.target_world_y = None

        self.charging_stations = {
            'amr_01': (14.7, -10.6),
            'amr_02': (14.7, -8.0),
            'amr_03': (14.7, -5.4),
        }

        self.robot_state = 'IDLE'
        self.pose_time = 0.0
        self.scan_time = 0.0
        self.pallet_pose = None
        self.pallet_time = 0.0
        self.pallet_yaw = 0.0
        self.attachment_state = None
        self.attachment_time = 0.0
        self.action_started = 0.0
        self.last_attachment_command = 0.0
        self.completed_tasks = set()
        self.last_replan = 0.0
        self.task_started = None
        self.task_start_battery = None
        self.distance_traveled = 0.0
        self.reroute_count = 0
        self.obstacle_count = 0
        self.attachment_reference = None
        self.carry_verified = False
        self.task_kind = 'pickup'
        self.declare_parameter('pickup_capable', self.robot_id == 'amr_02')
        self.pickup_capable = bool(self.get_parameter('pickup_capable').value)
        self.bootstrap_ready = not self.pickup_capable
        self.rear_obstacle_distance = float('inf')
        self.release_reference = None
        self.carry_relative = None
        self.last_progress_log = 0.0


        # =====================================================
        # A* NAVIGATION
        # =====================================================

        self.warehouse_map = self.MAP_CLASS()
        self.astar = AStarPlanner(
            self.warehouse_map
        )

        # Local-odometry waypoint queue.
        self.waypoints = []

        self.current_waypoint_index = 0

        self.goal_x = None
        self.goal_y = None

        self.goal_tolerance = 0.20

        # =====================================================
        # LIDAR
        # =====================================================

        self.scan_received = False

        self.front_obstacle_distance = float('inf')

        self.obstacle_stop_distance = 0.80
        self.obstacle_resume_distance = 1.10

        self.obstacle_detected = False

        # =====================================================
        # GAZEBO TOPICS
        # =====================================================

        self.cmd_vel_topic = (
            f'/model/{self.robot_id}/cmd_vel'
        )

        self.odom_topic = (
            f'/model/{self.robot_id}/odometry'
        )

        self.scan_topic = (
            f'/model/{self.robot_id}/scan'
        )

        self.pose_topic = (
            f'/model/{self.robot_id}/pose'
        )

        # =====================================================
        # ROS PUBLISHERS / SUBSCRIBERS
        # =====================================================

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        self.pose_received = False

        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            10
        )

        for peer_id in self.ROBOT_SPAWNS:
            if peer_id != self.robot_id:
                self.create_subscription(PoseStamped, f'/model/{peer_id}/pose',
                    lambda msg, rid=peer_id: self.peer_pose_callback(msg, rid), 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )

        self.assignment_sub = self.create_subscription(
            String,
            f'/{self.robot_id}/assigned_task',
            self.assigned_task_callback,
            10
        )

        self.battery_pub = self.create_publisher(
            Float32,
            f'/{self.robot_id}/battery',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            f'/{self.robot_id}/status',
            10
        )

        self.attach_pub = self.create_publisher(
            Empty,
            f'/{self.robot_id}/product_01/attach',
            10
        )

        self.detach_pub = self.create_publisher(
            Empty,
            f'/{self.robot_id}/product_01/detach',
            10
        )

        self.create_subscription(String, f'/{self.robot_id}/product_01/attached',
                                 self.attachment_callback, 10)
        self.create_subscription(PoseStamped, '/model/pallet_box_mobile/pose',
                                 self.pallet_callback, 10)
        self.telemetry_pub = self.create_publisher(String, f'/{self.robot_id}/telemetry', 10)
        self.result_pub = self.create_publisher(String, '/task/result', 10)
        self.path_pub = self.create_publisher(String, f'/{self.robot_id}/path_intention', 10)

        # =====================================================
        # TIMERS
        # =====================================================

        self.control_timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.battery_timer = self.create_timer(
            1.0,
            self.update_battery
        )

        self.status_timer = self.create_timer(
            1.0,
            self.publish_status
        )

        # =====================================================
        # STARTUP
        # =====================================================

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            f'{self.robot_id.upper()} controller started'
        )

        self.get_logger().info(
            f'Spawn world: '
            f'({self.spawn_x:.2f}, {self.spawn_y:.2f})'
        )

        self.get_logger().info(
            f'CMD_VEL: {self.cmd_vel_topic}'
        )

        self.get_logger().info(
            f'ODOMETRY: {self.odom_topic}'
        )

        self.get_logger().info(
            f'LIDAR: {self.scan_topic}'
        )

        self.get_logger().info(
            f'Initial Battery: '
            f'{self.battery_level:.1f}%'
        )

        self.get_logger().info(
            f'Obstacle safety: STOP <= '
            f'{self.obstacle_stop_distance:.2f} m | '
            f'RESUME >= '
            f'{self.obstacle_resume_distance:.2f} m'
        )

        self.get_logger().info(
            'A* waypoint navigation ENABLED'
        )

        self.get_logger().info(
            '========================================'
        )

    # =========================================================
    # WORLD POSE & ODOMETRY
    # =========================================================

    def pose_callback(self, msg):
        if msg.header.frame_id != self.WORLD_FRAME:
            return
        p = msg.pose
        if not all(math.isfinite(v) for v in (p.position.x, p.position.y,
                                             p.orientation.z, p.orientation.w)):
            return
        if self.pose_received and self.current_task_id:
            self.distance_traveled += math.hypot(p.position.x - self.x, p.position.y - self.y)
        self.x, self.y = p.position.x, p.position.y
        q = p.orientation
        self.yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.pose_received = True
        self.pose_time = time.monotonic()
        self.pose_sim_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def peer_pose_callback(self, msg, robot_id):
        if msg.header.frame_id == self.WORLD_FRAME:
            self.warehouse_map.peer_poses[robot_id] = (msg.pose.position.x, msg.pose.position.y)

    def odom_callback(self, msg):
        # Diagnostic only. Never silently reinterpret odometry as world pose.
        self.odom_received = True

    # =========================================================
    # LIDAR
    # =========================================================

    def scan_callback(self, msg):
        if not msg.ranges or not self.pose_received:
            return
        self.scan_received = True
        self.scan_time = time.monotonic()
        hits = []
        rear_hits = []
        for index, distance in enumerate(msg.ranges):
            angle = msg.angle_min + index * msg.angle_increment
            if not math.isfinite(distance):
                continue
            if not msg.range_min <= distance <= msg.range_max:
                continue
            wx = self.x + distance * math.cos(self.yaw + angle)
            wy = self.y + distance * math.sin(self.yaw + angle)
            # Exclude only returns on the measured target footprint, never all LiDAR.
            if self.pallet_pose and self.robot_state in ('DOCKING', 'PICKING', 'VERIFYING_CARRY', 'MOVING_TO_DELIVERY', 'DROPPING', 'VERIFYING_RELEASE'):
                dx, dy = wx-self.pallet_pose[0], wy-self.pallet_pose[1]
                px = dx*math.cos(self.pallet_yaw) + dy*math.sin(self.pallet_yaw)
                py = -dx*math.sin(self.pallet_yaw) + dy*math.cos(self.pallet_yaw)
                if abs(px) <= 0.66 and abs(py) <= 0.45:
                    continue
            if abs(angle) <= math.radians(40) or self.robot_state == 'MOVING_TO_DELIVERY':
                hits.append((distance, wx, wy))
            if abs(angle) >= math.radians(140):
                rear_hits.append(distance)
        self.rear_obstacle_distance = min(rear_hits, default=float('inf'))
        nearest = min(hits, default=(float('inf'), 0, 0))
        self.front_obstacle_distance = nearest[0]
        stop_distance = 1.85 if self.robot_state == 'MOVING_TO_DELIVERY' else self.obstacle_stop_distance
        self.obstacle_detected = nearest[0] <= stop_distance
        if self.obstacle_detected and self.robot_state.startswith('MOVING'):
            self.stop_robot()
            if time.monotonic() - self.last_replan > 2.0:
                self.obstacle_count += 1
                self.trigger_reroute(nearest)

    # =========================================================
    # DYNAMIC REROUTING
    # =========================================================

    def trigger_reroute(self, hit):
        if self.target_world_x is None or self.target_world_y is None:
            return
        self.last_replan = time.monotonic()
        _, x, y = hit
        self.warehouse_map.add_dynamic_obstacle(x, y, size=0.2)
        self.get_logger().warning(f'{self.robot_id} OBSTACLE DETECTED; REPLANNING')
        if self.set_route((self.target_world_x, self.target_world_y), self.robot_state):
            self.reroute_count += 1
            self.get_logger().info(f'{self.robot_id} REROUTE SUCCESSFUL')
        else:
            self.stop_robot()

    # =========================================================
    # TASK ASSIGNMENT
    # =========================================================

    def assigned_task_callback(self, msg):
        try:
            task = json.loads(msg.data)
            task_id = str(task['task_id'])
            pickup = (float(task['pickup_x']), float(task['pickup_y']))
            delivery = (float(task.get('delivery_x', -5.0)), float(task.get('delivery_y', 10.0)))
            if not task_id or not all(math.isfinite(v) for v in pickup + delivery):
                raise ValueError('Non-finite task coordinate')
        except (ValueError, KeyError, TypeError):
            self.get_logger().warning('Invalid task assignment')
            return
        if not self.bootstrap_ready or self.robot_state != 'IDLE' or self.current_task_id or task_id in self.completed_tasks:
            return
        if not self.pose_received or time.monotonic()-self.pose_time > 2 or self.battery_level <= 20:
            self.get_logger().warning('Task rejected: localization unavailable or low battery')
            return
        self.task_kind = task.get('kind', 'pickup')
        if self.task_kind != 'navigate' and (not self.pickup_capable or task.get('gazebo_model', 'pallet_box_mobile') != 'pallet_box_mobile'):
            self.get_logger().warning('Task rejected: attachment capability unavailable')
            return
        self.current_task_id = task_id
        self.task_started = time.monotonic()
        self.task_start_battery = self.battery_level
        self.distance_traveled = self.reroute_count = self.obstacle_count = 0
        self.carry_verified = False
        self.pickup_world_x, self.pickup_world_y = pickup
        self.delivery_world_x, self.delivery_world_y = delivery
        self.get_logger().info(f'{self.robot_id} TASK RECEIVED: {task_id} | CURRENT WORLD POSE={(self.x, self.y, self.yaw)} | PRODUCT CENTER={pickup}')
        if self.task_kind == 'navigate':
            if not self.set_route(pickup, 'MOVING_TO_GOAL'):
                self.fail_task('No navigation route')
            return
        if not self.pallet_pose or time.monotonic()-self.pallet_time > 2 or math.dist(pickup, self.pallet_pose) > 0.35:
            self.fail_task('Pallet pose missing or differs from task; reset PRODUCT01 first')
            return
        try:
            departure_map = self.MAP_CLASS(robot_inflation=1.85)
            departure_map.pallets = dict(self.warehouse_map.pallets)
            departure_map.pallets.pop('pallet_box_mobile', None)
            departure_map.peer_poses = dict(self.warehouse_map.peer_poses)
            goal, path = choose_approach(
                self.astar, (self.x, self.y), pickup, self.pallet_yaw,
                departure_planner=AStarPlanner(departure_map))
        except ValueError as error:
            self.fail_task(str(error))
            return
        self.approach_world_x, self.approach_world_y = goal
        self.get_logger().info(f'SELECTED APPROACH={goal}')
        if math.dist(goal, (self.x, self.y)) < 0.01:
            self.transition('DOCKING', 'Reached pickup approach')
            self.stop_robot()
        else:
            self.set_route(goal, 'MOVING_TO_PICKUP', path)

    # =========================================================
    # BATTERY
    # =========================================================

    def update_battery(self):
        if self.robot_state.startswith('MOVING') or self.robot_state == 'DOCKING':
            self.battery_level -= 0.02
        elif self.robot_state == 'IDLE':
            self.battery_level -= 0.002
        elif self.robot_state == 'CHARGING':
            self.battery_level = min(100.0, self.battery_level+0.5)
            if self.battery_level >= 100:
                self.transition('IDLE', 'Battery charged')
        self.battery_level = max(0.0, self.battery_level)
        if self.robot_state == 'IDLE' and self.battery_level <= 20 and self.pose_received:
            if not self.set_route(self.charging_stations[self.robot_id], 'MOVING_TO_CHARGER'):
                self.transition('LOW_BATTERY', 'No safe route to charger')

    # =========================================================
    # STATUS
    # =========================================================

    def publish_status(self):
        self.telemetry_pub.publish(String(data=json.dumps({'robot_id': self.robot_id,
            'x': self.x, 'y': self.y, 'yaw': self.yaw, 'state': self.robot_state,
            'bootstrap_ready': self.bootstrap_ready,
            'current_task': self.current_task_id, 'goal': [self.goal_x, self.goal_y],
            'obstacle': self.obstacle_detected, 'reroutes': self.reroute_count,
            'distance': self.distance_traveled, 'attachment': self.attachment_state,
            'carry_verified': self.carry_verified, 'localization': self.LOCALIZATION_SOURCE})))

        battery_msg = Float32()

        battery_msg.data = float(
            self.battery_level
        )

        self.battery_pub.publish(
            battery_msg
        )

        status_msg = String()

        status_msg.data = (
            self.robot_state
        )

        self.status_pub.publish(
            status_msg
        )

    # =========================================================
    # CONTROL LOOP
    # =========================================================

    def control_loop(self):
        now = time.monotonic()
        if not self.pose_received or now-self.pose_time > 2 or now-self.scan_time > 2:
            self.stop_robot()
            return
        if not self.bootstrap_ready:
            self.stop_robot()
            if self.attachment_state == 'detached':
                self.bootstrap_ready = True
                self.get_logger().info('Startup joint detached; pickup enabled')
            elif now-self.last_attachment_command > 1:
                self.detach_pub.publish(Empty())
                self.last_attachment_command = now
            return
        if self.robot_state in ('VERIFYING_CARRY', 'VERIFYING_RELEASE'):
            self.verify_motion(now)
            return
        if self.robot_state in ('PICKING', 'DROPPING'):
            self.attachment_step(now)
            return
        if self.robot_state == 'DOCKING':
            self.docking_step()
            return
        if not self.robot_state.startswith('MOVING'):
            return
        if self.robot_state == 'MOVING_TO_DELIVERY':
            if self.attachment_state != 'attached' or now-self.pallet_time > 2:
                self.fail_task('Attachment or pallet localization lost', holding=True)
                return
            if not self.relative_geometry_valid():
                self.fail_task('Attached pallet relative geometry changed', holding=True)
                return
        if self.goal_x is None:
            if now-self.last_replan > 2 and self.target_world_x is not None:
                self.last_replan = now
                self.set_route((self.target_world_x, self.target_world_y), self.robot_state)
            self.stop_robot()
            return
        self.move_to_goal()

    # =========================================================
    # MOVE TO CURRENT WAYPOINT
    # =========================================================

    def move_to_goal(self):
        dx, dy = self.goal_x-self.x, self.goal_y-self.y
        distance = math.hypot(dx, dy)
        if distance < self.goal_tolerance:
            self.stop_robot()
            if self.current_waypoint_index + 1 < len(self.waypoints):
                self.current_waypoint_index += 1
                self.goal_x, self.goal_y = self.waypoints[self.current_waypoint_index]
                self.get_logger().info(f'CURRENT WAYPOINT {self.current_waypoint_index}: {(self.goal_x, self.goal_y)}')
                return
            self.goal_x = self.goal_y = None
            if self.robot_state == 'MOVING_TO_PICKUP':
                self.transition('DOCKING', 'Reached pickup approach')
            elif self.robot_state == 'MOVING_TO_DELIVERY':
                if not self.carry_verified:
                    self.fail_task('Delivery reached without meaningful verified carry', holding=True)
                    return
                self.begin_attachment_action('DROPPING')
            elif self.robot_state == 'MOVING_TO_CHARGER':
                self.transition('CHARGING', 'Reached charger')
            elif self.robot_state == 'MOVING_TO_GOAL':
                self.finish_task()
            return
        angle = self.normalize_angle(math.atan2(dy, dx)-self.yaw)
        cmd = Twist()
        # Rotation may clear an obstacle from the front sector; translation remains inhibited.
        if abs(angle) > 0.20:
            cmd.angular.z = max(-0.5, min(0.5, 1.2*angle))
        elif not self.obstacle_detected:
            cmd.linear.x = min(0.35, 0.5*distance)
            cmd.angular.z = max(-0.30, min(0.30, angle))
        self.cmd_vel_pub.publish(cmd)

    # =========================================================
    # STOP
    # =========================================================

    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(
            cmd
        )

    # =========================================================
    # NORMALIZE ANGLE
    # =========================================================

    @staticmethod
    def normalize_angle(angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def pallet_callback(self, msg):
        if msg.header.frame_id != self.WORLD_FRAME:
            return
        p = msg.pose
        if not all(math.isfinite(v) for v in (p.position.x, p.position.y)):
            return
        self.pallet_pose = (p.position.x, p.position.y)
        q = p.orientation
        self.pallet_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        self.pallet_time = time.monotonic()
        if self.attachment_state != 'attached':
            self.warehouse_map.pallets['pallet_box_mobile'] = (*self.pallet_pose, self.pallet_yaw)

    def attachment_callback(self, msg):
        if msg.data in ('attached', 'detached'):
            self.attachment_state = msg.data
            self.attachment_time = time.monotonic()

    def set_route(self, goal, state, path=None):
        try:
            path = path or self.astar.plan((self.x, self.y), goal)
        except ValueError as error:
            self.get_logger().warning(str(error))
            path = None
        if not path:
            self.waypoints = []
            self.goal_x = self.goal_y = None
            return False
        self.target_world_x, self.target_world_y = goal
        self.waypoints = path[1:] if len(path) > 1 else path
        self.current_waypoint_index = 0
        self.goal_x, self.goal_y = self.waypoints[0]
        self.transition(state, 'Safe route selected')
        self.path_pub.publish(String(data=json.dumps({'robot_id': self.robot_id,
            'timestamp': time.time(), 'path': path, 'task_id': self.current_task_id,
            'nominal_speed': 0.35, 'coordination': 'safety_stop_only'})))
        self.get_logger().info(f'{self.robot_id} A* PATH READY; {state}; A* PATH LENGTH={sum(math.dist(a,b) for a,b in zip(path,path[1:])):.3f}; WORLD WAYPOINTS={path}; CURRENT WAYPOINT={(self.goal_x, self.goal_y)}')
        return True

    def docking_step(self):
        if not self.pallet_pose or time.monotonic()-self.pallet_time > 2:
            self.stop_robot()
            return
        dx, dy = self.pallet_pose[0]-self.x, self.pallet_pose[1]-self.y
        distance = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)
        angle = self.normalize_angle(bearing-self.yaw)
        cmd = Twist()
        if self.obstacle_detected or distance < 0.78:
            self.stop_robot()
            return
        if abs(angle) > 0.08:
            cmd.angular.z = max(-0.25, min(0.25, angle))
        elif distance <= docking_distance(bearing, self.pallet_yaw) + 0.03:
            self.begin_attachment_action('PICKING')
            return
        else:
            cmd.linear.x = min(0.08, 0.3*(distance-docking_distance(bearing, self.pallet_yaw)))
        self.cmd_vel_pub.publish(cmd)

    def begin_attachment_action(self, state):
        self.stop_robot()
        self.transition(state, 'Docked' if state == 'PICKING' else 'Reached receiving location')
        self.action_started = time.monotonic()
        self.last_attachment_command = 0.0
        if state == 'DROPPING':
            self.release_settle_pose = (self.x, self.y, *self.pallet_pose)
            self.release_settle_since = self.action_started
        self.get_logger().info(f'{self.robot_id} {state}; awaiting Gazebo acknowledgment')

    def attachment_step(self, now):
        self.stop_robot()
        expected = 'attached' if self.robot_state == 'PICKING' else 'detached'
        if self.attachment_state == expected and self.attachment_time > self.action_started:
            if expected == 'attached':
                if not self.pallet_pose:
                    self.fail_task('Pallet pose unavailable after attachment', holding=True)
                    return
                self.attachment_reference = (self.x, self.y, *self.pallet_pose)
                self.warehouse_map.pallets.pop('pallet_box_mobile', None)
                self.get_logger().info(f'{self.robot_id} PRODUCT ATTACHED (Gazebo acknowledgment)')
                self.carry_relative = self.pallet_relative()
                self.action_started = now
                self.motion_started_sim = self.pose_sim_time
                self.transition('VERIFYING_CARRY', 'Joint acknowledged; prove rigid transport before delivery')
                self.get_logger().info(f'ATTACH BASELINE robot={(self.x,self.y,self.yaw)} pallet={self.pallet_pose} relative={self.carry_relative}')
            else:
                self.get_logger().info(f'{self.robot_id} PRODUCT DETACHED (Gazebo acknowledgment)')
                self.release_reference = (self.x, self.y, *self.pallet_pose)
                self.action_started = now
                self.motion_started_sim = self.pose_sim_time
                self.transition('VERIFYING_RELEASE', 'Joint detached; prove pallet stays at receiving location')
            return
        # Free rollers retain momentum: keep the joint while the pair stops.
        # Use world displacement rather than wheel odometry to prove rest.
        if self.robot_state == 'DROPPING' and self.last_attachment_command == 0.0:
            current = (self.x, self.y, *self.pallet_pose)
            reference = self.release_settle_pose
            if (math.dist(current[:2], reference[:2]) > 0.005 or
                    math.dist(current[2:], reference[2:]) > 0.005):
                self.release_settle_pose = current
                self.release_settle_since = now
            if now-self.release_settle_since < 2.0:
                if now-self.action_started > 20:
                    self.fail_task('Load did not settle before release', holding=True)
                return
            self.action_started = now
        if now-self.action_started > 10:
            self.fail_task(f'No {expected} acknowledgment', holding=True)
            return
        if now-self.last_attachment_command > 1:
            publisher = self.attach_pub if expected == 'attached' else self.detach_pub
            publisher.publish(Empty())
            self.last_attachment_command = now

    def finish_task(self):
        self.stop_robot()
        task_id = self.current_task_id
        result = {'task_id': task_id, 'robot_id': self.robot_id, 'status': 'COMPLETED',
                  'duration': time.monotonic()-self.task_started,
                  'distance': self.distance_traveled, 'reroutes': self.reroute_count,
                  'obstacles': self.obstacle_count, 'carry_verified': self.carry_verified,
                  'energy': self.task_start_battery-self.battery_level}
        self.result_pub.publish(String(data=json.dumps(result)))
        self.completed_tasks.add(task_id)
        self.current_task_id = None
        self.waypoints = []
        self.goal_x = self.goal_y = None
        self.warehouse_map = self.MAP_CLASS()
        if self.pallet_pose:
            self.warehouse_map.pallets['pallet_box_mobile'] = (*self.pallet_pose, self.pallet_yaw)
        self.astar = AStarPlanner(self.warehouse_map)
        self.transition('COMPLETED', 'Physical workflow verified')
        self.transition('IDLE', 'Task released')
        self.target_world_x = self.target_world_y = None
        self.get_logger().info(f'{self.robot_id} TASK COMPLETED: {task_id}')

    def fail_task(self, reason, holding=False):
        self.stop_robot()
        self.result_pub.publish(String(data=json.dumps({'task_id': self.current_task_id,
            'robot_id': self.robot_id, 'status': 'FAILED', 'reason': reason})))
        self.get_logger().error(f'{self.robot_id} TASK FAILED: {reason}')
        self.transition('ERROR' if holding else 'IDLE', reason)
        self.current_task_id = None
        self.goal_x = self.goal_y = None
        self.waypoints = []


    def transition(self, state, reason):
        if self.robot_state != state:
            self.get_logger().info(f'{self.robot_state} -> {state} | {reason}')
            self.robot_state = state

    def pallet_relative(self):
        dx, dy = self.pallet_pose[0]-self.x, self.pallet_pose[1]-self.y
        return (dx*math.cos(self.yaw)+dy*math.sin(self.yaw),
                -dx*math.sin(self.yaw)+dy*math.cos(self.yaw),
                self.normalize_angle(self.pallet_yaw-self.yaw))

    def relative_geometry_valid(self):
        if self.carry_relative is None or self.pallet_pose is None:
            return False
        current = self.pallet_relative()
        return (math.dist(current[:2], self.carry_relative[:2]) < 0.15 and
                abs(self.normalize_angle(current[2]-self.carry_relative[2])) < 0.15)

    def verify_motion(self, now):
        carrying = self.robot_state == 'VERIFYING_CARRY'
        reference = self.attachment_reference if carrying else self.release_reference
        if now-self.pallet_time > 2 or reference is None:
            self.fail_task('Pallet localization unavailable for physical verification', holding=True)
            return
        if carrying and (self.attachment_state != 'attached' or not self.relative_geometry_valid()):
            self.fail_task('Attachment failed rigid geometry check', holding=True)
            return
        rx, ry, px, py = reference
        travel = math.hypot(self.x-rx, self.y-ry)
        pallet_travel = math.dist(self.pallet_pose, (px,py))
        if not carrying and pallet_travel > 0.15:
            self.fail_task('Pallet followed robot after detach', holding=True)
            return
        # Back straight out of the station before turning with the load.
        required = 1.4 if carrying else 0.4
        if travel >= required:
            self.stop_robot()
            self.get_logger().info(f'PHYSICAL {"CARRY" if carrying else "RELEASE"} VERIFIED robot_delta={travel:.3f} pallet_delta={pallet_travel:.3f} robot={(self.x,self.y)} pallet={self.pallet_pose}')
            if carrying:
                if pallet_travel < required-0.15:
                    self.fail_task('Pallet did not follow robot', holding=True)
                    return
                self.carry_verified = True
                loaded_map = self.MAP_CLASS(robot_inflation=1.85)
                loaded_map.pallets = dict(self.warehouse_map.pallets)
                loaded_map.peer_poses = dict(self.warehouse_map.peer_poses)
                loaded_map.pallets.pop('pallet_box_mobile', None)
                self.warehouse_map = loaded_map
                self.astar = AStarPlanner(loaded_map)
                if not self.set_route((self.delivery_world_x,self.delivery_world_y), 'MOVING_TO_DELIVERY'):
                    self.fail_task('No delivery route', holding=True)
            else:
                self.finish_task()
            return
        if self.pose_sim_time-self.motion_started_sim > 90:
            self.fail_task('Physical verification motion timed out', holding=True)
            return
        cmd = Twist()
        if self.rear_obstacle_distance > self.obstacle_stop_distance:
            cmd.linear.x = -0.08
        self.cmd_vel_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    controller = AMRController()

    try:

        rclpy.spin(
            controller
        )

    except KeyboardInterrupt:

        pass

    finally:

        if rclpy.ok():
            controller.stop_robot()

        controller.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
