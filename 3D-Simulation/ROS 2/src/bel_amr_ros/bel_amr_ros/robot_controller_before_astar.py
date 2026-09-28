#!/usr/bin/env python3

import json
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String


class AMRController(Node):

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

        # =====================================================
        # ROBOT POSITION / STATE
        # =====================================================

        # These are Gazebo DiffDrive LOCAL odometry coordinates.
        # They are continuously updated from odometry and are
        # NEVER reset when a new goal is received.

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.goal_x = None
        self.goal_y = None

        self.current_task_id = None

        self.robot_state = 'IDLE'

        self.odom_received = False

        # =====================================================
        # LIDAR / OBSTACLE DETECTION
        # =====================================================

        self.scan_received = False

        # Closest valid obstacle inside the front sector.
        self.front_obstacle_distance = float('inf')

        # Stop if an obstacle is <= 0.80 m away.
        self.obstacle_stop_distance = 0.80

        # Resume only when the front is >= 1.10 m clear.
        # Hysteresis prevents rapid STOP/GO switching.
        self.obstacle_resume_distance = 1.10

        self.obstacle_detected = False

        # =====================================================
        # GAZEBO TOPICS
        # =====================================================

        self.cmd_vel_topic = f'/model/{self.robot_id}/cmd_vel'
        self.odom_topic = f'/model/{self.robot_id}/odometry'
        self.scan_topic = f'/model/{self.robot_id}/scan'

        # =====================================================
        # VELOCITY PUBLISHER
        # =====================================================

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        # =====================================================
        # ODOMETRY SUBSCRIBER
        # =====================================================

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        # =====================================================
        # LIDAR SUBSCRIBER
        # =====================================================

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )

        # =====================================================
        # LOCAL P2P AGENT -> CONTROLLER
        # =====================================================

        self.assignment_sub = self.create_subscription(
            String,
            f'/{self.robot_id}/assigned_task',
            self.assigned_task_callback,
            10
        )

        # =====================================================
        # BATTERY + STATUS PUBLISHERS
        # =====================================================

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

        # =====================================================
        # TIMERS
        # =====================================================

        # Main movement controller - 10 Hz
        self.control_timer = self.create_timer(
            0.1,
            self.control_loop
        )

        # Battery simulation - 1 Hz
        self.battery_timer = self.create_timer(
            1.0,
            self.update_battery
        )

        # Robot status publisher - 1 Hz
        self.status_timer = self.create_timer(
            1.0,
            self.publish_status
        )

        # =====================================================
        # STARTUP INFORMATION
        # =====================================================

        self.get_logger().info(
            f'{self.robot_id.upper()} controller started'
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
            f'ASSIGNED TASK: /{self.robot_id}/assigned_task'
        )

        self.get_logger().info(
            f'Initial Battery: {self.battery_level:.1f}%'
        )

        self.get_logger().info(
            f'Obstacle safety: STOP <= '
            f'{self.obstacle_stop_distance:.2f} m | '
            f'RESUME >= {self.obstacle_resume_distance:.2f} m'
        )

    # =========================================================
    # ODOMETRY CALLBACK
    # =========================================================

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Quaternion -> yaw
        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        self.yaw = math.atan2(
            siny_cosp,
            cosy_cosp
        )

        self.odom_received = True

    # =========================================================
    # LIDAR CALLBACK
    # =========================================================

    def scan_callback(self, msg):

        if not msg.ranges:
            return

        self.scan_received = True

        # -----------------------------------------------------
        # FRONT SAFETY SECTOR
        #
        # LaserScan angle 0 is straight ahead.
        # We monitor approximately -30 to +30 degrees.
        # -----------------------------------------------------

        front_half_angle = math.radians(30.0)

        valid_ranges = []

        for index, distance in enumerate(msg.ranges):

            angle = (
                msg.angle_min +
                index * msg.angle_increment
            )

            if abs(angle) <= front_half_angle:

                if (
                    math.isfinite(distance)
                    and msg.range_min <= distance <= msg.range_max
                ):
                    valid_ranges.append(distance)

        if valid_ranges:
            minimum_distance = min(valid_ranges)
        else:
            minimum_distance = float('inf')

        self.front_obstacle_distance = minimum_distance

        # -----------------------------------------------------
        # OBSTACLE APPEARED
        # -----------------------------------------------------

        if (
            not self.obstacle_detected
            and minimum_distance
            <= self.obstacle_stop_distance
        ):

            self.obstacle_detected = True

            self.get_logger().warning(
                f'{self.robot_id.upper()} '
                f'OBSTACLE DETECTED | '
                f'Front distance='
                f'{minimum_distance:.2f} m'
            )

            # Immediate safety stop.
            self.stop_robot()

        # -----------------------------------------------------
        # OBSTACLE CLEARED
        # -----------------------------------------------------

        elif (
            self.obstacle_detected
            and minimum_distance
            >= self.obstacle_resume_distance
        ):

            self.obstacle_detected = False

            self.get_logger().info(
                f'{self.robot_id.upper()} '
                f'PATH CLEAR | '
                f'Front distance='
                f'{minimum_distance:.2f} m'
            )

    # =========================================================
    # RECEIVE ASSIGNED TASK FROM LOCAL P2P AGENT
    # =========================================================

    def assigned_task_callback(self, msg):

        try:

            task = json.loads(msg.data)

            task_id = str(
                task['task_id']
            )

            # P2P converts warehouse/world pickup position
            # into this robot's LOCAL odometry coordinates.

            goal_x = float(
                task['local_goal_x']
            )

            goal_y = float(
                task['local_goal_y']
            )

            pickup_x = float(
                task.get(
                    'pickup_x',
                    0.0
                )
            )

            pickup_y = float(
                task.get(
                    'pickup_y',
                    0.0
                )
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
        # IGNORE DUPLICATE ASSIGNMENT
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
        # WAIT FOR ODOMETRY
        # -----------------------------------------------------

        if not self.odom_received:

            self.get_logger().warning(
                f'{self.robot_id.upper()} '
                f'cannot execute {task_id}: '
                f'odometry not received yet'
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
        # ACCEPT ASSIGNMENT
        # -----------------------------------------------------

        self.current_task_id = task_id

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            f'{self.robot_id.upper()} '
            f'TASK RECEIVED: {task_id}'
        )

        self.get_logger().info(
            f'Warehouse target: '
            f'({pickup_x:.2f}, {pickup_y:.2f})'
        )

        self.get_logger().info(
            f'Local odom target: '
            f'({goal_x:.2f}, {goal_y:.2f})'
        )

        self.get_logger().info(
            f'Current local pose: '
            f'({self.x:.2f}, {self.y:.2f}) | '
            f'Yaw='
            f'{math.degrees(self.yaw):.1f} deg'
        )

        self.get_logger().info(
            '========================================'
        )

        success = self.set_goal(
            goal_x,
            goal_y
        )

        if not success:
            self.current_task_id = None

    # =========================================================
    # BATTERY SIMULATION
    # =========================================================

    def update_battery(self):

        # This remains the current temporary battery model.
        # Full automatic charging will be added after
        # waypoint navigation is implemented.

        if self.robot_state == 'MOVING':

            self.battery_level -= 0.02

        elif self.robot_state == 'IDLE':

            self.battery_level -= 0.002

        elif self.robot_state == 'CHARGING':

            self.battery_level += 0.10

        # Keep battery inside [0, 100].

        self.battery_level = max(
            0.0,
            min(
                100.0,
                self.battery_level
            )
        )

        # -----------------------------------------------------
        # LOW BATTERY PROTECTION
        # -----------------------------------------------------

        if (
            self.battery_level <= 20.0
            and self.robot_state
            not in ['CHARGING', 'LOW_BATTERY']
        ):

            self.robot_state = 'LOW_BATTERY'

            self.stop_robot()

            self.get_logger().warning(
                f'{self.robot_id.upper()} '
                f'LOW BATTERY: '
                f'{self.battery_level:.1f}%'
            )

    # =========================================================
    # PUBLISH ROBOT STATUS
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
    # MAIN CONTROL LOOP
    # =========================================================

    def control_loop(self):

        if not self.odom_received:
            return

        if self.robot_state != 'MOVING':
            return

        if self.goal_x is None:
            return

        if self.goal_y is None:
            return

        # -----------------------------------------------------
        # LIDAR SAFETY
        # -----------------------------------------------------

        if (
            self.scan_received
            and self.obstacle_detected
        ):

            # Do NOT cancel the task.
            # Do NOT change robot_state to IDLE.
            # Simply stop movement while the path is blocked.

            self.stop_robot()
            return

        # -----------------------------------------------------
        # NORMAL MOVEMENT
        # -----------------------------------------------------

        self.move_to_goal()

    # =========================================================
    # MOVE TO GOAL
    # =========================================================

    def move_to_goal(self):

        # ALWAYS calculate movement from CURRENT odometry.

        dx = (
            self.goal_x -
            self.x
        )

        dy = (
            self.goal_y -
            self.y
        )

        distance = math.hypot(
            dx,
            dy
        )

        # -----------------------------------------------------
        # GOAL REACHED
        # -----------------------------------------------------

        if distance < 0.20:

            completed_task = (
                self.current_task_id
            )

            self.stop_robot()

            self.robot_state = 'IDLE'

            self.goal_x = None
            self.goal_y = None

            self.current_task_id = None

            self.get_logger().info(
                '========================================'
            )

            self.get_logger().info(
                f'{self.robot_id.upper()} '
                f'reached goal'
            )

            if completed_task is not None:

                self.get_logger().info(
                    f'TASK {completed_task} '
                    f'PICKUP LOCATION REACHED'
                )

            self.get_logger().info(
                f'Final local pose: '
                f'({self.x:.2f}, {self.y:.2f})'
            )

            self.get_logger().info(
                '========================================'
            )

            return

        # -----------------------------------------------------
        # DIRECTION TO GOAL
        # -----------------------------------------------------

        target_angle = math.atan2(
            dy,
            dx
        )

        angle_error = (
            self.normalize_angle(
                target_angle -
                self.yaw
            )
        )

        cmd = Twist()

        # -----------------------------------------------------
        # TURN TOWARD GOAL
        # -----------------------------------------------------

        if abs(angle_error) > 0.20:

            # Turn in place.

            cmd.linear.x = 0.0

            cmd.angular.z = max(
                -0.5,
                min(
                    0.5,
                    1.2 * angle_error
                )
            )

        # -----------------------------------------------------
        # DRIVE TOWARD GOAL
        # -----------------------------------------------------

        else:

            # Forward velocity decreases near target.

            cmd.linear.x = min(
                0.35,
                0.5 * distance
            )

            # Small heading correction.

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
    # SET GOAL
    # =========================================================

    def set_goal(self, x, y):

        if self.battery_level <= 20.0:

            self.get_logger().warning(
                f'{self.robot_id.upper()} '
                f'cannot accept goal. '
                f'Battery: '
                f'{self.battery_level:.1f}%'
            )

            return False

        if not self.odom_received:

            self.get_logger().warning(
                f'{self.robot_id.upper()} '
                f'cannot accept goal: '
                f'odometry unavailable'
            )

            return False

        # IMPORTANT:
        #
        # Only the goal changes.
        # x, y and yaw remain the robot's real current
        # local odometry pose.

        self.goal_x = float(x)
        self.goal_y = float(y)

        self.robot_state = 'MOVING'

        self.get_logger().info(
            f'{self.robot_id.upper()} '
            f'NEW GOAL | '
            f'Current='
            f'({self.x:.2f}, {self.y:.2f}) | '
            f'Goal='
            f'({self.goal_x:.2f}, '
            f'{self.goal_y:.2f}) | '
            f'Yaw='
            f'{math.degrees(self.yaw):.1f} deg'
        )

        return True

    # =========================================================
    # STOP ROBOT
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