#!/usr/bin/env python3

import json
import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String

from .edge_learning import CostModel, features
from rclpy.qos import QoSProfile, DurabilityPolicy
from .lacam_pibt import ConflictDetector, PIBTResolver


class P2PAgent(Node):
    WORLD_FRAME = 'world_demo'


    def __init__(self):
        super().__init__('p2p_agent')

        # =========================================================
        # ROBOT CONFIGURATION
        # =========================================================

        self.declare_parameter('robot_id', 'amr_01')

        self.robot_id = (
            self.get_parameter('robot_id')
            .get_parameter_value()
            .string_value
        )

        self.robot_ids = [
            'amr_01',
            'amr_02',
            'amr_03'
        ]

        # Warehouse/world spawn positions.
        self.spawn_positions = {
            'amr_01': (13.9, -10.6),
            'amr_02': (13.9, -8.0),
            'amr_03': (13.9, -5.4)
        }

        self.spawn_x, self.spawn_y = self.spawn_positions.get(
            self.robot_id,
            (0.0, 0.0)
        )

        # =========================================================
        # LOCAL ROBOT STATE
        # =========================================================

        # Gazebo DiffDrive local odometry.
        self.local_x = 0.0
        self.local_y = 0.0

        # Warehouse/world position.
        self.x = self.spawn_x
        self.y = self.spawn_y

        self.battery = 100.0
        self.controller_status = 'IDLE'

        # Task allocation state.
        self.allocation_status = 'IDLE'
        self.current_task = None

        # =========================================================
        # DISTRIBUTED INFORMATION
        # =========================================================

        # robot_id -> peer state
        self.peer_states = {}

        # task_id -> task information
        self.tasks = {}

        # task_id -> {robot_id -> bid information}
        self.task_bids = {}

        # task_id -> winner robot_id
        self.task_claims = {}

        # Tasks already evaluated locally.
        self.finalized_tasks = set()

        # Edge-AI local model instance
        self.learned_model = None
        self.model_version = 0
        self.pose_time = 0.0
        self.pending_features = {}
        self.pending_bid_task = None
        self.declare_parameter('enable_ai', True)
        self.declare_parameter('pickup_capable', self.robot_id == 'amr_02')
        self.pickup_capable = self.get_parameter('pickup_capable').value
        self.create_subscription(String, '/task/result', self.result_callback, 10)
        model_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, f'/{self.robot_id}/learned_model', self.model_callback, model_qos)
        self.completed_task_count = 0

        # =========================================================
        # DDS / ROS 2 PUBLISHERS
        # =========================================================

        self.state_pub = self.create_publisher(
            String,
            f'/{self.robot_id}/peer_state',
            10
        )

        self.bid_pub = self.create_publisher(
            String,
            f'/{self.robot_id}/task_bid',
            10
        )

        self.claim_pub = self.create_publisher(
            String,
            f'/{self.robot_id}/task_claim',
            10
        )

        # ---------------------------------------------------------
        # LOCAL P2P AGENT -> LOCAL ROBOT CONTROLLER
        # ---------------------------------------------------------

        self.assignment_pub = self.create_publisher(
            String,
            f'/{self.robot_id}/assigned_task',
            10
        )

        # =========================================================
        # OWN ROBOT SUBSCRIPTIONS
        # =========================================================

        self.pose_received = False

        self.create_subscription(
            PoseStamped,
            f'/model/{self.robot_id}/pose',
            self.pose_callback,
            10
        )

        self.create_subscription(
            Odometry,
            f'/model/{self.robot_id}/odometry',
            self.odom_callback,
            10
        )

        self.create_subscription(
            Float32,
            f'/{self.robot_id}/battery',
            self.battery_callback,
            10
        )

        self.create_subscription(
            String,
            f'/{self.robot_id}/status',
            self.status_callback,
            10
        )

        # =========================================================
        # WAREHOUSE TASK ANNOUNCEMENT
        # =========================================================

        self.create_subscription(
            String,
            '/task/announcement',
            self.task_callback,
            10
        )

        # =========================================================
        # DDS PEER STATE SUBSCRIPTIONS
        # =========================================================

        for peer_id in self.robot_ids:

            if peer_id == self.robot_id:
                continue

            self.create_subscription(
                String,
                f'/{peer_id}/peer_state',
                lambda msg, pid=peer_id:
                    self.peer_state_callback(msg, pid),
                10
            )

        # =========================================================
        # DDS BID SUBSCRIPTIONS
        # =========================================================

        # Subscribe to ALL bid topics, including our own.
        # This gives every agent the same distributed bid table.

        for peer_id in self.robot_ids:

            self.create_subscription(
                String,
                f'/{peer_id}/task_bid',
                lambda msg, pid=peer_id:
                    self.bid_callback(msg, pid),
                10
            )

        # =========================================================
        # DDS CLAIM SUBSCRIPTIONS
        # =========================================================

        for peer_id in self.robot_ids:

            if peer_id == self.robot_id:
                continue

            self.create_subscription(
                String,
                f'/{peer_id}/task_claim',
                lambda msg, pid=peer_id:
                    self.claim_callback(msg, pid),
                10
            )

        # =========================================================
        # TIMERS
        # =========================================================

        # Broadcast state once per second.
        self.create_timer(
            1.0,
            self.broadcast_state
        )

        # Evaluate pending task bids twice per second.
        self.create_timer(
            0.5,
            self.evaluate_tasks
        )

        # =========================================================
        # STARTUP INFORMATION
        # =========================================================

        self.get_logger().info(
            f'{self.robot_id.upper()} '
            f'decentralized P2P agent started'
        )

        self.get_logger().info(
            f'Warehouse spawn position: '
            f'({self.spawn_x:.2f}, {self.spawn_y:.2f})'
        )

        self.get_logger().info(
            f'Assignment topic: '
            f'/{self.robot_id}/assigned_task'
        )

    # =============================================================
    # WORLD POSE & ODOMETRY
    # =============================================================

    def pose_callback(self, msg):

        if msg.header.frame_id != self.WORLD_FRAME:
            return
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.pose_received = True
        self.pose_time = time.monotonic()

    def odom_callback(self, msg):
        # Retain odometry for diagnostics only; bidding requires fresh world pose.
        self.local_x = msg.pose.pose.position.x
        self.local_y = msg.pose.pose.position.y

    # =============================================================
    # BATTERY
    # =============================================================

    def battery_callback(self, msg):

        self.battery = float(msg.data)

    # =============================================================
    # CONTROLLER STATUS
    # =============================================================

    def status_callback(self, msg):
        self.controller_status = msg.data
        # Task completion comes from an explicit task/result event, not an inferred state transition.

    # =============================================================
    # ROBOT AVAILABILITY
    # =============================================================

    def is_available(self):
        return (self.allocation_status == 'IDLE' and self.current_task is None
                and self.pending_bid_task is None
                and self.battery > 20.0 and self.controller_status == 'IDLE'
                and self.pose_received and time.monotonic()-self.pose_time < 2)

    # =============================================================
    # BROADCAST LOCAL STATE THROUGH ROS 2 / DDS
    # =============================================================

    def broadcast_state(self):

        state = {
            'robot_id': self.robot_id,
            'x': round(self.x, 3),
            'y': round(self.y, 3),
            'local_x': round(self.local_x, 3),
            'local_y': round(self.local_y, 3),
            'battery': round(self.battery, 2),
            'controller_status': self.controller_status,
            'allocation_status': self.allocation_status,
            'available': self.is_available(),
            'model_version': self.model_version,
            'current_task': self.current_task
        }

        msg = String()
        msg.data = json.dumps(state)

        self.state_pub.publish(msg)

    # =============================================================
    # RECEIVE PEER STATE
    # =============================================================

    def peer_state_callback(self, msg, peer_id):

        try:

            state = json.loads(msg.data)

            self.peer_states[peer_id] = state

        except json.JSONDecodeError:

            self.get_logger().warning(
                f'Invalid peer state from {peer_id}'
            )

    # =============================================================
    # RECEIVE WAREHOUSE TASK
    # =============================================================

    def task_callback(self, msg):

        try:

            task = json.loads(msg.data)

            task_id = str(task['task_id'])

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
                'Invalid task announcement'
            )

            return

        # Ignore duplicate task announcements.

        if task_id in self.tasks:
            return

        self.tasks[task_id] = task
        self.task_bids.setdefault(task_id, {})

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            f'NEW TASK {task_id} | '
            f'Pickup=({pickup_x:.2f}, {pickup_y:.2f})'
        )

        # ---------------------------------------------------------
        # Busy robots still participate by sending a zero bid.
        #
        # This allows all AMRs to receive one response from every
        # robot and independently reach the same result.
        # ---------------------------------------------------------

        available = self.is_available() and (task.get('kind') == 'navigate' or self.pickup_capable)

        if not available:

            bid_value = 0.0

            self.get_logger().info(
                f'{self.robot_id.upper()} unavailable for '
                f'{task_id} | '
                f'Current task={self.current_task} | '
                f'State={self.allocation_status}'
            )

        else:

            bid_value = self.calculate_bid(
                pickup_x,
                pickup_y
            )

        bid_data = {
            'task_id': task_id,
            'robot_id': self.robot_id,
            'bid': round(bid_value, 4),
            'battery': round(self.battery, 2),
            'available': available,
            'x': round(self.x, 3),
            'y': round(self.y, 3),
            'timestamp': time.time()
        }

        # Reserve locally before another announcement callback can bid this AMR again.
        if available:
            self.pending_bid_task = task_id

        # Store our own bid immediately.
        self.task_bids[task_id][self.robot_id] = bid_data

        # Broadcast bid through ROS 2 / DDS.
        bid_msg = String()
        bid_msg.data = json.dumps(
            bid_data
        )

        self.bid_pub.publish(
            bid_msg
        )

        self.get_logger().info(
            f'{self.robot_id.upper()} BID for '
            f'{task_id}: {bid_value:.4f}'
        )

    # =============================================================
    # LOCAL SUITABILITY / BID CALCULATION
    # =============================================================

    def calculate_bid(self, pickup_x, pickup_y):
        distance = math.hypot(pickup_x-self.x, pickup_y-self.y)
        deterministic = 0.5*max(0.0, 1-distance/20) + 0.3*max(0.0, min(1.0, self.battery/100)) + 0.2
        suitability = None
        final = deterministic
        if self.get_parameter('enable_ai').value and self.learned_model is not None:
            try:
                suitability = 1-self.learned_model.predict(features(self.battery, distance))
                final = 0.9*deterministic + 0.1*suitability
            except (ValueError, TypeError, OverflowError):
                self.get_logger().warning('Model unavailable; deterministic bidding fallback')
        self.get_logger().info(f'{self.robot_id} BID deterministic={deterministic:.4f} '
                               f'AI suitability={suitability} final={final:.4f} model={self.model_version}')
        return final

    # =============================================================
    # RECEIVE DDS BID
    # =============================================================

    def bid_callback(self, msg, peer_id):
        try:
            bid = json.loads(msg.data)
            task_id = str(bid['task_id'])
            if bid['robot_id'] != peer_id or peer_id not in self.robot_ids:
                return
            value = float(bid['bid'])
            if not math.isfinite(value) or not 0 <= value <= 1 or not isinstance(bid['available'], bool):
                return
            self.task_bids.setdefault(task_id, {}).setdefault(peer_id, bid)
        except (ValueError, KeyError, TypeError):
            self.get_logger().warning('Rejected malformed bid')

    # =============================================================
    # DECENTRALIZED WINNER SELECTION
    # =============================================================

    def evaluate_tasks(self):

        for task_id in list(
            sorted(self.task_bids.keys())
        ):

            if task_id not in self.tasks or task_id in self.finalized_tasks:
                continue

            bids = self.task_bids[task_id]

            # Wait until one bid has arrived from every AMR.
            if len(bids) < len(self.robot_ids):
                continue

            # -----------------------------------------------------
            # FILTER VALID BIDS
            # -----------------------------------------------------

            valid_bids = [
                bid
                for bid in bids.values()
                if (
                    bool(
                        bid.get(
                            'available',
                            False
                        )
                    )
                    and float(
                        bid.get(
                            'bid',
                            0.0
                        )
                    ) > 0.0
                )
            ]

            if not valid_bids:

                self.finalized_tasks.add(
                    task_id
                )

                self.get_logger().warning(
                    f'NO AVAILABLE ROBOT for {task_id}'
                )

                continue

            # -----------------------------------------------------
            # DECENTRALIZED DETERMINISTIC WINNER
            #
            # Highest bid wins.
            #
            # robot_id is used as deterministic tie-breaker.
            #
            # Every AMR performs this calculation locally.
            # -----------------------------------------------------

            sorted_bids = sorted(
                valid_bids,
                key=lambda item: (
                    -float(item['bid']),
                    item['robot_id']
                )
            )

            winner = sorted_bids[0]

            winner_id = winner['robot_id']
            if self.pending_bid_task == task_id:
                self.pending_bid_task = None
            winning_bid = float(
                winner['bid']
            )

            self.finalized_tasks.add(
                task_id
            )

            self.task_claims[
                task_id
            ] = winner_id

            self.get_logger().info(
                '========================================'
            )

            self.get_logger().info(
                f'DECENTRALIZED RESULT: {task_id}'
            )

            for bid in sorted(
                bids.values(),
                key=lambda item: item['robot_id']
            ):

                self.get_logger().info(
                    f"{bid['robot_id'].upper()} | "
                    f"Bid={bid['bid']} | "
                    f"Available="
                    f"{bid.get('available', False)}"
                )

            self.get_logger().info(
                f'WINNER: {winner_id.upper()}'
            )

            # -----------------------------------------------------
            # THIS ROBOT WON
            # -----------------------------------------------------

            if winner_id == self.robot_id:

                # Check availability again immediately before
                # accepting the task.

                if not self.is_available():

                    self.get_logger().warning(
                        f'{self.robot_id.upper()} was selected '
                        f'for {task_id} but is no longer available'
                    )

                    self.get_logger().info(
                        '========================================'
                    )

                    continue

                # -------------------------------------------------
                # ACCEPT TASK LOCALLY
                # -------------------------------------------------

                self.current_task = task_id
                self.allocation_status = 'ASSIGNED'

                task = self.tasks.get(
                    task_id
                )

                if task is None:

                    self.get_logger().error(
                        f'Task information missing for {task_id}'
                    )

                    self.current_task = None
                    self.allocation_status = 'IDLE'

                    continue

                pickup_x = float(
                    task['pickup_x']
                )

                pickup_y = float(
                    task['pickup_y']
                )

                # -------------------------------------------------
                # COORDINATE CONVERSION
                #
                # Warehouse/world target:
                #     pickup_x, pickup_y
                #
                # Gazebo DiffDrive odometry:
                #     local coordinates starting at (0,0)
                #
                # Since the current AMRs spawn with yaw = 0:
                #
                # local_goal = warehouse_goal - spawn_position
                #
                # IMPORTANT:
                # This does NOT reset the robot.
                # It only expresses the fixed warehouse target in
                # the robot's local odometry frame.
                # -------------------------------------------------

                # -------------------------------------------------
                # SEND ASSIGNMENT TO LOCAL CONTROLLER
                # -------------------------------------------------

                assignment = dict(task)
                assignment['learning_features'] = features(self.battery, math.hypot(pickup_x-self.x, pickup_y-self.y))

                assignment_msg = String()

                assignment_msg.data = json.dumps(
                    assignment
                )

                self.assignment_pub.publish(
                    assignment_msg
                )

                self.get_logger().info(
                    f'{self.robot_id.upper()} '
                    f'EXECUTION COMMAND -> {task_id}'
                )

                self.get_logger().info(
                    f'Warehouse goal = '
                    f'({pickup_x:.2f}, {pickup_y:.2f})'
                )

                # -------------------------------------------------
                # BROADCAST CLAIM TO PEER AMRs
                # -------------------------------------------------

                claim = {
                    'task_id': task_id,
                    'robot_id': self.robot_id,
                    'bid': winning_bid,
                    'status': 'CLAIMED'
                }

                claim_msg = String()

                claim_msg.data = json.dumps(
                    claim
                )

                self.claim_pub.publish(
                    claim_msg
                )

                self.get_logger().info(
                    f'{self.robot_id.upper()} '
                    f'CLAIMED {task_id}'
                )

                self.get_logger().info(
                    f'{self.robot_id.upper()} '
                    f'STATE -> ASSIGNED'
                )

            # -----------------------------------------------------
            # THIS ROBOT LOST
            # -----------------------------------------------------

            else:

                self.get_logger().info(
                    f'{self.robot_id.upper()} '
                    f'RELEASED {task_id}'
                )

            self.get_logger().info(
                '========================================'
            )

    # =============================================================
    # RECEIVE DDS CLAIM
    # =============================================================

    def claim_callback(
        self,
        msg,
        peer_id
    ):

        try:

            claim = json.loads(
                msg.data
            )

            task_id = str(
                claim['task_id']
            )

            winner = str(
                claim['robot_id']
            )

            self.task_claims[
                task_id
            ] = winner

            self.get_logger().info(
                f'DDS CLAIM received: '
                f'{winner.upper()} owns {task_id}'
            )

        except (
            json.JSONDecodeError,
            KeyError,
            TypeError
        ):

            self.get_logger().warning(
                f'Invalid claim received from {peer_id}'
            )


    def model_callback(self, msg):
        try:
            data = json.loads(msg.data)
            model = CostModel(data['weights'])
            if data['robot_id'] != self.robot_id or data['version'] <= self.model_version or model.digest() != data['digest']:
                return
            self.learned_model = model
            self.model_version = data['version']
        except (ValueError, KeyError, TypeError):
            self.get_logger().warning('Rejected invalid learned model; retaining fallback')

    def result_callback(self, msg):
        try:
            event = json.loads(msg.data)
            task_id = event['task_id']
            if event['status'] not in ('COMPLETED', 'FAILED'):
                return
            if self.task_claims.get(task_id) != event['robot_id']:
                return
            self.task_claims.pop(task_id, None)
            if self.current_task == task_id:
                self.current_task = None
                self.allocation_status = 'IDLE'
                if event['status'] == 'COMPLETED':
                    self.completed_task_count += 1
        except (ValueError, KeyError, TypeError):
            return


def main(args=None):

    rclpy.init(
        args=args
    )

    node = P2PAgent()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
