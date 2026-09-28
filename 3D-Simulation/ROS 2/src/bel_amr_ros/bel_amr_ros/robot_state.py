#!/usr/bin/env python3

import time


class RobotState:
    """
    Unified state representation for an EdgeFleet-AI AMR.

    This object stores the information that an AMR shares with
    peer robots through ROS 2 / DDS.

    It is designed to support:
    - Decentralized P2P coordination
    - Conflict detection
    - CBS-style safety
    - Deadlock detection
    - GNN risk prediction
    - MARL coordination
    - Dashboard monitoring
    """

    def __init__(self, robot_id):

        # =====================================================
        # ROBOT IDENTITY
        # =====================================================

        self.robot_id = robot_id

        # =====================================================
        # WORLD POSITION
        # =====================================================

        self.x = 0.0
        self.y = 0.0

        # =====================================================
        # LOCAL ODOMETRY POSITION
        # =====================================================

        self.local_x = 0.0
        self.local_y = 0.0

        # =====================================================
        # ORIENTATION
        # =====================================================

        self.yaw = 0.0

        # =====================================================
        # VELOCITY
        # =====================================================

        self.vx = 0.0
        self.vy = 0.0

        # =====================================================
        # BATTERY
        # =====================================================

        self.battery = 100.0

        # =====================================================
        # ROBOT STATUS
        # =====================================================

        self.controller_status = 'IDLE'

        self.allocation_status = 'IDLE'

        # =====================================================
        # TASK
        # =====================================================

        self.current_task = None

        # =====================================================
        # NAVIGATION GOAL
        # =====================================================

        self.goal_x = None
        self.goal_y = None

        # =====================================================
        # NAVIGATION INTENT
        # =====================================================

        self.next_waypoint = None

        self.planned_path = []

        # =====================================================
        # COORDINATION
        # =====================================================

        self.priority = 0

        # =====================================================
        # SAFETY
        # =====================================================

        self.obstacle_detected = False

        # =====================================================
        # TIMESTAMP
        # =====================================================

        self.timestamp = time.time()

    # =========================================================
    # UPDATE TIMESTAMP
    # =========================================================

    def update_timestamp(self):

        self.timestamp = time.time()

    # =========================================================
    # AVAILABILITY
    # =========================================================

    def is_available(self):

        return (
            self.allocation_status == 'IDLE'
            and self.current_task is None
            and self.battery > 20.0
        )

    # =========================================================
    # CONVERT STATE TO DICTIONARY
    # =========================================================

    def to_dict(self):

        self.update_timestamp()

        return {
            'robot_id':
                self.robot_id,

            'x':
                round(
                    float(self.x),
                    3
                ),

            'y':
                round(
                    float(self.y),
                    3
                ),

            'local_x':
                round(
                    float(self.local_x),
                    3
                ),

            'local_y':
                round(
                    float(self.local_y),
                    3
                ),

            'yaw':
                round(
                    float(self.yaw),
                    4
                ),

            'vx':
                round(
                    float(self.vx),
                    3
                ),

            'vy':
                round(
                    float(self.vy),
                    3
                ),

            'battery':
                round(
                    float(self.battery),
                    2
                ),

            'controller_status':
                self.controller_status,

            'allocation_status':
                self.allocation_status,

            'available':
                self.is_available(),

            'current_task':
                self.current_task,

            'goal_x':
                self.goal_x,

            'goal_y':
                self.goal_y,

            'next_waypoint':
                self.next_waypoint,

            'planned_path':
                self.planned_path,

            'priority':
                self.priority,

            'obstacle_detected':
                self.obstacle_detected,

            'timestamp':
                self.timestamp
        }

    # =========================================================
    # CREATE STATE FROM RECEIVED P2P DICTIONARY
    # =========================================================

    @classmethod
    def from_dict(
        cls,
        data
    ):

        robot_id = str(
            data.get(
                'robot_id',
                'unknown'
            )
        )

        state = cls(
            robot_id
        )

        # =====================================================
        # POSITION
        # =====================================================

        state.x = cls._safe_float(
            data.get(
                'x',
                0.0
            )
        )

        state.y = cls._safe_float(
            data.get(
                'y',
                0.0
            )
        )

        state.local_x = cls._safe_float(
            data.get(
                'local_x',
                0.0
            )
        )

        state.local_y = cls._safe_float(
            data.get(
                'local_y',
                0.0
            )
        )

        # =====================================================
        # ORIENTATION
        # =====================================================

        state.yaw = cls._safe_float(
            data.get(
                'yaw',
                0.0
            )
        )

        # =====================================================
        # VELOCITY
        # =====================================================

        state.vx = cls._safe_float(
            data.get(
                'vx',
                0.0
            )
        )

        state.vy = cls._safe_float(
            data.get(
                'vy',
                0.0
            )
        )

        # =====================================================
        # BATTERY
        # =====================================================

        state.battery = cls._safe_float(
            data.get(
                'battery',
                100.0
            )
        )

        # =====================================================
        # STATUS
        # =====================================================

        state.controller_status = str(
            data.get(
                'controller_status',
                'IDLE'
            )
        )

        state.allocation_status = str(
            data.get(
                'allocation_status',
                'IDLE'
            )
        )

        # =====================================================
        # TASK
        # =====================================================

        state.current_task = data.get(
            'current_task'
        )

        # =====================================================
        # GOAL
        # =====================================================

        goal_x = data.get(
            'goal_x'
        )

        goal_y = data.get(
            'goal_y'
        )

        if goal_x is not None:

            state.goal_x = cls._safe_float(
                goal_x
            )

        if goal_y is not None:

            state.goal_y = cls._safe_float(
                goal_y
            )

        # =====================================================
        # NAVIGATION INTENT
        # =====================================================

        next_waypoint = data.get(
            'next_waypoint'
        )

        if (
            isinstance(
                next_waypoint,
                (list, tuple)
            )
            and len(next_waypoint) >= 2
        ):

            state.next_waypoint = [
                cls._safe_float(
                    next_waypoint[0]
                ),
                cls._safe_float(
                    next_waypoint[1]
                )
            ]

        else:

            state.next_waypoint = None

        planned_path = data.get(
            'planned_path',
            []
        )

        state.planned_path = []

        if isinstance(
            planned_path,
            list
        ):

            for point in planned_path:

                if (
                    isinstance(
                        point,
                        (list, tuple)
                    )
                    and len(point) >= 2
                ):

                    state.planned_path.append(
                        [
                            cls._safe_float(
                                point[0]
                            ),
                            cls._safe_float(
                                point[1]
                            )
                        ]
                    )

        # =====================================================
        # PRIORITY
        # =====================================================

        try:

            state.priority = int(
                data.get(
                    'priority',
                    0
                )
            )

        except (
            TypeError,
            ValueError
        ):

            state.priority = 0

        # =====================================================
        # OBSTACLE STATUS
        # =====================================================

        state.obstacle_detected = bool(
            data.get(
                'obstacle_detected',
                False
            )
        )

        # =====================================================
        # TIMESTAMP
        # =====================================================

        state.timestamp = cls._safe_float(
            data.get(
                'timestamp',
                time.time()
            )
        )

        return state

    # =========================================================
    # SAFE FLOAT CONVERSION
    # =========================================================

    @staticmethod
    def _safe_float(
        value,
        default=0.0
    ):

        try:

            return float(
                value
            )

        except (
            TypeError,
            ValueError
        ):

            return float(
                default
            )

    # =========================================================
    # DISTANCE TO ANOTHER ROBOT
    # =========================================================

    def distance_to(
        self,
        other_state
    ):

        dx = (
            other_state.x
            - self.x
        )

        dy = (
            other_state.y
            - self.y
        )

        return (
            dx * dx
            + dy * dy
        ) ** 0.5

    # =========================================================
    # STATE AGE
    # =========================================================

    def age(self):

        return max(
            0.0,
            time.time()
            - self.timestamp
        )

    # =========================================================
    # DEBUG REPRESENTATION
    # =========================================================

    def __repr__(self):

        return (
            f'RobotState('
            f'robot_id={self.robot_id}, '
            f'position=({self.x:.2f}, {self.y:.2f}), '
            f'battery={self.battery:.1f}, '
            f'controller={self.controller_status}, '
            f'task={self.current_task}'
            f')'
        )