#!/usr/bin/env python3

import json
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String, Empty

from .warehouse_map import WarehouseMap
from .astar_planner import AStarPlanner


class AMRController(Node):

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

        # =====================================================
        # A* NAVIGATION
        # =====================================================

        self.warehouse_map = WarehouseMap()
        self.astar = AStarPlanner(
            self.warehouse_map
        )

        # Local-odometry waypoint queue.
        self.waypoints = []

        self.current_waypoint_index = 0

        self.goal_x = None
        self.goal_y = None

        self.goal_tolerance = 0.20

        # Temporary product approach offset.
        #
        # Product center:
        # (pickup_x, pickup_y)
        #
        # Approach from negative Y:
        # (pickup_x, pickup_y - 1.2952)
        #
        # For PRODUCT_01 this gives approximately:
        # (4.4161, 13.40)

        self.pickup_approach_offset = 1.2952

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

        self.x = msg.pose.position.x
        self.y = msg.pose.position.y

        q = msg.pose.orientation

        siny_cosp = 2.0 * (
            q.w * q.z
            + q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y
            + q.z * q.z
        )

        self.yaw = math.atan2(
            siny_cosp,
            cosy_cosp
        )

        self.pose_received = True
        self.odom_received = True

    def odom_callback(self, msg):

        if self.pose_received:
            return

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (
            q.w * q.z
            + q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y
            + q.z * q.z
        )

        self.yaw = math.atan2(
            siny_cosp,
            cosy_cosp
        )

        self.odom_received = True

    # =========================================================
    # LIDAR
    # =========================================================

    def scan_callback(self, msg):

        if not msg.ranges:
            return

        self.scan_received = True

        front_half_angle = math.radians(
            30.0
        )

        valid_ranges = []

        for index, distance in enumerate(
            msg.ranges
        ):

            angle = (
                msg.angle_min
                + index * msg.angle_increment
            )

            if abs(angle) <= front_half_angle:

                if (
                    math.isfinite(distance)
                    and msg.range_min
                    <= distance
                    <= msg.range_max
                ):
                    valid_ranges.append(
                        distance
                    )

        if valid_ranges:
            minimum_distance = min(
                valid_ranges
            )
        else:
            minimum_distance = float('inf')

        self.front_obstacle_distance = (
            minimum_distance
        )

        if (
            not self.obstacle_detected
            and minimum_distance <= self.obstacle_stop_distance
            and self.robot_state in ['MOVING_TO_PICKUP', 'MOVING_TO_DELIVERY']
        ):

            self.obstacle_detected = True

            self.get_logger().warning(
                f'{self.robot_id.upper()} '
                f'OBSTACLE DETECTED | '
                f'Front distance='
                f'{minimum_distance:.2f} m'
            )

            self.stop_robot()
            self.trigger_reroute(minimum_distance)

        elif (
            self.obstacle_detected
            and minimum_distance >= self.obstacle_resume_distance
        ):

            self.obstacle_detected = False

            self.get_logger().info(
                f'{self.robot_id.upper()} '
                f'PATH CLEAR | '
                f'Front distance='
                f'{minimum_distance:.2f} m'
            )

    # =========================================================
    # DYNAMIC REROUTING
    # =========================================================

    def trigger_reroute(self, obstacle_distance):

        if self.target_world_x is None or self.target_world_y is None:
            return

        obs_x = self.x + obstacle_distance * math.cos(self.yaw)
        obs_y = self.y + obstacle_distance * math.sin(self.yaw)

        self.get_logger().warning(
            f'========================================\n'
            f'{self.robot_id.upper()} DYNAMIC OBSTACLE DETECTED at ({obs_x:.2f}, {obs_y:.2f})\n'
            f'Inserting obstacle into WarehouseMap and replanning A* route...\n'
            f'========================================'
        )

        self.warehouse_map.add_dynamic_obstacle(obs_x, obs_y)

        try:
            new_path = self.astar.plan(
                (self.x, self.y),
                (self.target_world_x, self.target_world_y)
            )
        except Exception as error:
            self.get_logger().error(f'A* replan exception: {error}')
            new_path = None

        if new_path:
            if len(new_path) > 1:
                self.waypoints = new_path[1:]
            else:
                self.waypoints = new_path

            self.current_waypoint_index = 0
            self.goal_x = self.waypoints[0][0]
            self.goal_y = self.waypoints[0][1]
            self.obstacle_detected = False

            self.get_logger().info(
                f'{self.robot_id.upper()} DYNAMIC REROUTE SUCCESSFUL! '
                f'New path has {len(self.waypoints)} waypoints. Resuming movement.'
            )
        else:
            self.get_logger().warning(
                f'{self.robot_id.upper()} REROUTE FAILED: No alternate path available around obstacle.'
            )

    # =========================================================
    # TASK ASSIGNMENT
    # =========================================================

    def assigned_task_callback(self, msg):

        try:

            task = json.loads(
                msg.data
            )

            task_id = str(
                task['task_id']
            )

            pickup_x = float(
                task['pickup_x']
            )

            pickup_y = float(
                task['pickup_y']
            )

        except (
            json.JSONDecodeError,
            KeyError,
            TypeError,
            ValueError
        ):

            self.get_logger().warning(
                f'{self.robot_id.upper()} '
                f'received invalid assigned task'
            )

            return

        # -----------------------------------------------------
        # DUPLICATE TASK
        # -----------------------------------------------------

        if (
            self.current_task_id == task_id
            and self.robot_state == 'MOVING'
        ):

            self.get_logger().info(
                f'{self.robot_id.upper()} '
                f'already executing {task_id}'
            )

            return

        # -----------------------------------------------------
        # ODOMETRY CHECK
        # -----------------------------------------------------

        if not self.odom_received:

            self.get_logger().warning(
                f'{self.robot_id.upper()} '
                f'cannot execute {task_id}: '
                f'odometry unavailable'
            )

            return

        # -----------------------------------------------------
        # BATTERY CHECK
        # -----------------------------------------------------

        if self.battery_level <= 20.0:

            self.get_logger().warning(
                f'{self.robot_id.upper()} '
                f'cannot execute {task_id}: '
                f'low battery '
                f'({self.battery_level:.1f}%)'
            )

            return

        # -----------------------------------------------------
        # STORE TASK
        # -----------------------------------------------------

        self.current_task_id = task_id

        self.pickup_world_x = pickup_x
        self.pickup_world_y = pickup_y

        self.delivery_world_x = float(task.get('delivery_x', -5.0))
        self.delivery_world_y = float(task.get('delivery_y', 10.0))

        # -----------------------------------------------------
        # CALCULATE PRODUCT APPROACH & TARGET GOAL
        # -----------------------------------------------------

        self.approach_world_x = pickup_x
        self.approach_world_y = pickup_y - self.pickup_approach_offset

        self.target_world_x = self.approach_world_x
        self.target_world_y = self.approach_world_y

        # -----------------------------------------------------
        # CURRENT WORLD POSITION
        # -----------------------------------------------------

        current_world_x = self.x
        current_world_y = self.y

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            f'{self.robot_id.upper()} '
            f'TASK RECEIVED: {task_id}'
        )

        self.get_logger().info(
            f'Current WORLD pose: '
            f'({current_world_x:.2f}, '
            f'{current_world_y:.2f})'
        )

        self.get_logger().info(
            f'Product center: '
            f'({pickup_x:.2f}, '
            f'{pickup_y:.2f})'
        )

        self.get_logger().info(
            f'Product approach: '
            f'({self.approach_world_x:.2f}, '
            f'{self.approach_world_y:.2f})'
        )

        # -----------------------------------------------------
        # A* PLAN
        # -----------------------------------------------------

        try:

            world_path = self.astar.plan(
                (
                    current_world_x,
                    current_world_y
                ),
                (
                    self.target_world_x,
                    self.target_world_y
                )
            )

        except ValueError as error:

            self.get_logger().error(
                f'A* planning failed for '
                f'{task_id}: {error}'
            )

            self.current_task_id = None
            self.robot_state = 'IDLE'

            return

        if not world_path:

            self.get_logger().error(
                f'{self.robot_id.upper()} '
                f'A* found no route for '
                f'{task_id}'
            )

            self.current_task_id = None
            self.robot_state = 'IDLE'

            return

        # -----------------------------------------------------
        # STORE WORLD WAYPOINTS DIRECTLY
        # -----------------------------------------------------

        if len(world_path) > 1:
            self.waypoints = world_path[1:]
        else:
            self.waypoints = world_path

        if not self.waypoints:

            self.get_logger().warning(
                f'{self.robot_id.upper()} '
                f'has no navigation waypoints'
            )

            self.current_task_id = None
            self.robot_state = 'IDLE'

            return

        self.current_waypoint_index = 0

        self.goal_x = (
            self.waypoints[0][0]
        )

        self.goal_y = (
            self.waypoints[0][1]
        )

        self.robot_state = 'MOVING_TO_PICKUP'

        self.get_logger().info(
            f'A* PATH READY | '
            f'{len(world_path)} world points | '
            f'{len(self.waypoints)} navigation waypoints'
        )

        for index, point in enumerate(
            world_path
        ):

            self.get_logger().info(
                f'WORLD WP {index}: '
                f'({point[0]:.2f}, '
                f'{point[1]:.2f})'
            )

        self.get_logger().info(
            f'NAVIGATING TO LOCAL WP 1/'
            f'{len(self.waypoints)} | '
            f'({self.goal_x:.2f}, '
            f'{self.goal_y:.2f})'
        )

        self.get_logger().info(
            '========================================'
        )

    # =========================================================
    # BATTERY
    # =========================================================

    def update_battery(self):

        if self.robot_state in ['MOVING_TO_PICKUP', 'MOVING_TO_DELIVERY', 'MOVING_TO_CHARGER']:

            self.battery_level -= 0.02

        elif self.robot_state == 'IDLE':

            self.battery_level -= 0.002

        elif self.robot_state == 'CHARGING':

            self.battery_level += 0.50
            if self.battery_level >= 100.0:
                self.battery_level = 100.0
                self.robot_state = 'IDLE'
                self.get_logger().info(
                    f'{self.robot_id.upper()} CHARGING COMPLETE (100%). Returned to IDLE.'
                )

        self.battery_level = max(
            0.0,
            min(
                100.0,
                self.battery_level
            )
        )

        if (
            self.battery_level <= 20.0
            and self.robot_state not in ['CHARGING', 'MOVING_TO_CHARGER', 'LOW_BATTERY', 'MOVING_TO_PICKUP', 'MOVING_TO_DELIVERY']
        ):

            self.robot_state = 'LOW_BATTERY'

            self.stop_robot()

            self.get_logger().warning(
                f'{self.robot_id.upper()} LOW BATTERY: {self.battery_level:.1f}%'
            )

    # =========================================================
    # STATUS
    # =========================================================

    def publish_status(self):

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

        if not self.odom_received:
            return

        if self.robot_state not in ['MOVING_TO_PICKUP', 'MOVING_TO_DELIVERY', 'MOVING_TO_CHARGER']:
            return

        if self.goal_x is None or self.goal_y is None:
            return

        if (
            self.scan_received
            and self.obstacle_detected
        ):

            self.stop_robot()

            return

        self.move_to_goal()

    # =========================================================
    # MOVE TO CURRENT WAYPOINT
    # =========================================================

    def move_to_goal(self):

        dx = (
            self.goal_x
            - self.x
        )

        dy = (
            self.goal_y
            - self.y
        )

        distance = math.hypot(
            dx,
            dy
        )

        # -----------------------------------------------------
        # WAYPOINT REACHED
        # -----------------------------------------------------

        if distance < self.goal_tolerance:

            self.stop_robot()

            reached_number = (
                self.current_waypoint_index
                + 1
            )

            self.get_logger().info(
                f'{self.robot_id.upper()} '
                f'WAYPOINT REACHED '
                f'{reached_number}/'
                f'{len(self.waypoints)}'
            )

            # ---------------------------------------------
            # MORE WAYPOINTS REMAIN
            # ---------------------------------------------

            if (
                self.current_waypoint_index
                + 1
                < len(self.waypoints)
            ):

                self.current_waypoint_index += 1

                self.goal_x = (
                    self.waypoints[
                        self.current_waypoint_index
                    ][0]
                )

                self.goal_y = (
                    self.waypoints[
                        self.current_waypoint_index
                    ][1]
                )

                self.get_logger().info(
                    f'{self.robot_id.upper()} '
                    f'NEXT WAYPOINT '
                    f'{self.current_waypoint_index + 1}/'
                    f'{len(self.waypoints)} | '
                    f'Target='
                    f'({self.goal_x:.2f}, '
                    f'{self.goal_y:.2f})'
                )

                return

            # ---------------------------------------------
            # FINAL WAYPOINT OF CURRENT PHASE REACHED
            # ---------------------------------------------

            completed_task = self.current_task_id

            if self.robot_state == 'MOVING_TO_PICKUP':

                self.get_logger().info(
                    f'========================================\n'
                    f'{self.robot_id.upper()} PICKUP APPROACH REACHED!\n'
                    f'State -> PICKING | Attaching product...\n'
                    f'========================================'
                )

                self.robot_state = 'PICKING'
                self.attach_pub.publish(Empty())

                self.target_world_x = self.delivery_world_x
                self.target_world_y = self.delivery_world_y

                try:
                    world_path = self.astar.plan(
                        (self.x, self.y),
                        (self.target_world_x, self.target_world_y)
                    )
                except Exception as err:
                    self.get_logger().error(f'Delivery A* planning error: {err}')
                    world_path = None

                if world_path:
                    if len(world_path) > 1:
                        self.waypoints = world_path[1:]
                    else:
                        self.waypoints = world_path

                    self.current_waypoint_index = 0
                    self.goal_x = self.waypoints[0][0]
                    self.goal_y = self.waypoints[0][1]
                    self.robot_state = 'MOVING_TO_DELIVERY'

                    self.get_logger().info(
                        f'{self.robot_id.upper()} MOVING_TO_DELIVERY | '
                        f'Navigating to delivery point ({self.delivery_world_x:.2f}, {self.delivery_world_y:.2f})'
                    )
                else:
                    self.get_logger().error(f'{self.robot_id.upper()} Delivery path planning failed!')
                    self.robot_state = 'IDLE'

                return

            elif self.robot_state == 'MOVING_TO_DELIVERY':

                self.get_logger().info(
                    f'========================================\n'
                    f'{self.robot_id.upper()} DELIVERY LOCATION REACHED!\n'
                    f'State -> DROPPING | Detaching product...\n'
                    f'========================================'
                )

                self.robot_state = 'DROPPING'
                self.detach_pub.publish(Empty())

                self.get_logger().info(
                    f'========================================\n'
                    f'{self.robot_id.upper()} TASK {completed_task} COMPLETED SUCCESSFULLY!\n'
                    f'========================================'
                )

                self.current_task_id = None
                self.waypoints = []
                self.goal_x = None
                self.goal_y = None
                self.current_waypoint_index = 0

                if self.battery_level <= 20.0:
                    charger_x, charger_y = self.charging_stations.get(self.robot_id, (14.7, -10.6))
                    self.target_world_x = charger_x
                    self.target_world_y = charger_y

                    try:
                        world_path = self.astar.plan((self.x, self.y), (charger_x, charger_y))
                    except Exception as err:
                        world_path = None

                    if world_path:
                        if len(world_path) > 1:
                            self.waypoints = world_path[1:]
                        else:
                            self.waypoints = world_path
                        self.current_waypoint_index = 0
                        self.goal_x = self.waypoints[0][0]
                        self.goal_y = self.waypoints[0][1]
                        self.robot_state = 'MOVING_TO_CHARGER'
                        self.get_logger().warning(
                            f'{self.robot_id.upper()} LOW BATTERY ({self.battery_level:.1f}%). Navigating to charger.'
                        )
                    else:
                        self.robot_state = 'IDLE'
                else:
                    self.robot_state = 'IDLE'

                return

            elif self.robot_state == 'MOVING_TO_CHARGER':

                self.get_logger().info(
                    f'========================================\n'
                    f'{self.robot_id.upper()} CHARGING STATION REACHED!\n'
                    f'State -> CHARGING | Recharging battery...\n'
                    f'========================================'
                )

                self.robot_state = 'CHARGING'
                self.waypoints = []
                self.goal_x = None
                self.goal_y = None
                return

        # -----------------------------------------------------
        # TARGET HEADING
        # -----------------------------------------------------

        target_angle = math.atan2(
            dy,
            dx
        )

        angle_error = (
            self.normalize_angle(
                target_angle
                - self.yaw
            )
        )

        cmd = Twist()

        # -----------------------------------------------------
        # ROTATE
        # -----------------------------------------------------

        if abs(angle_error) > 0.20:

            cmd.linear.x = 0.0

            cmd.angular.z = max(
                -0.5,
                min(
                    0.5,
                    1.2 * angle_error
                )
            )

        # -----------------------------------------------------
        # DRIVE
        # -----------------------------------------------------

        else:

            cmd.linear.x = min(
                0.35,
                0.5 * distance
            )

            cmd.angular.z = max(
                -0.30,
                min(
                    0.30,
                    angle_error
                )
            )

        self.cmd_vel_pub.publish(
            cmd
        )

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

        controller.stop_robot()

        controller.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()