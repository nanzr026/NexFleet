#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
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

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.goal_x = None
        self.goal_y = None

        self.robot_state = 'IDLE'

        # =====================================================
        # GAZEBO TOPICS
        # =====================================================

        self.cmd_vel_topic = f'/model/{self.robot_id}/cmd_vel'
        self.odom_topic = f'/model/{self.robot_id}/odometry'

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

        # Robot status - 1 Hz
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
            f'Initial Battery: {self.battery_level:.1f}%'
        )

    # =========================================================
    # ODOMETRY CALLBACK
    # =========================================================

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

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

    # =========================================================
    # BATTERY SIMULATION
    # =========================================================

    def update_battery(self):

        # Temporary simulated battery model.
        # Later we will connect this with:
        # movement,
        # task execution,
        # charging stations.

        if self.robot_state == 'MOVING':

            # Faster discharge while moving
            self.battery_level -= 0.02

        elif self.robot_state == 'IDLE':

            # Very small idle consumption
            self.battery_level -= 0.002

        elif self.robot_state == 'CHARGING':

            # Temporary simulated charging
            self.battery_level += 0.10

        # Keep battery between 0 and 100
        self.battery_level = max(
            0.0,
            min(100.0, self.battery_level)
        )

        # Detect low battery
        if (
            self.battery_level <= 20.0
            and self.robot_state not in ['CHARGING', 'LOW_BATTERY']
        ):
            self.robot_state = 'LOW_BATTERY'

            self.get_logger().warning(
                f'{self.robot_id.upper()} LOW BATTERY: '
                f'{self.battery_level:.1f}%'
            )

    # =========================================================
    # PUBLISH ROBOT STATUS
    # =========================================================

    def publish_status(self):

        battery_msg = Float32()
        battery_msg.data = float(self.battery_level)

        self.battery_pub.publish(
            battery_msg
        )

        status_msg = String()
        status_msg.data = self.robot_state

        self.status_pub.publish(
            status_msg
        )

    # =========================================================
    # MAIN CONTROL LOOP
    # =========================================================

    def control_loop(self):

        if self.robot_state != 'MOVING':
            return

        if self.goal_x is None:
            return

        if self.goal_y is None:
            return

        self.move_to_goal()

    # =========================================================
    # MOVE TO GOAL
    # =========================================================

    def move_to_goal(self):

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y

        distance = math.hypot(
            dx,
            dy
        )

        target_angle = math.atan2(
            dy,
            dx
        )

        angle_error = self.normalize_angle(
            target_angle - self.yaw
        )

        # -----------------------------------------------------
        # GOAL REACHED
        # -----------------------------------------------------

        if distance < 0.20:

            self.stop_robot()

            self.robot_state = 'IDLE'

            self.goal_x = None
            self.goal_y = None

            self.get_logger().info(
                f'{self.robot_id.upper()} reached goal'
            )

            return

        cmd = Twist()

        # -----------------------------------------------------
        # TURN TOWARD GOAL
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
        # DRIVE TOWARD GOAL
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
    # SET GOAL
    # =========================================================

    def set_goal(self, x, y):

        if self.battery_level <= 20.0:

            self.get_logger().warning(
                f'{self.robot_id.upper()} cannot accept goal. '
                f'Battery: {self.battery_level:.1f}%'
            )

            return

        self.goal_x = float(x)
        self.goal_y = float(y)

        self.robot_state = 'MOVING'

        self.get_logger().info(
            f'{self.robot_id.upper()} goal: '
            f'({self.goal_x:.2f}, {self.goal_y:.2f})'
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