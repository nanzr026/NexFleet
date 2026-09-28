#!/usr/bin/env python3

import math


class ConflictDetector:
    """
    EdgeFleet-AI decentralized multi-AMR conflict detector.

    Detects:

    1. PROXIMITY conflicts
       Two robots are physically too close.

    2. VERTEX conflicts
       Two robots intend to occupy approximately the same
       location at approximately the same path step.

    3. EDGE conflicts
       Two robots intend to exchange positions / travel through
       the same path segment in opposite directions.

    This module DOES NOT stop or control robots.

    It only detects and reports conflicts.

    The CBS-style safety layer will later consume these conflict
    reports and decide whether an AMR should proceed, wait,
    yield, or replan.
    """

    # =========================================================
    # INITIALIZATION
    # =========================================================

    def __init__(
        self,
        safety_distance=1.20,
        vertex_distance=0.60,
        edge_distance=0.60,
        prediction_horizon=20
    ):

        # Minimum safe physical separation between AMRs.
        self.safety_distance = float(
            safety_distance
        )

        # Distance threshold used to determine whether two
        # predicted path points represent the same location.
        self.vertex_distance = float(
            vertex_distance
        )

        # Distance threshold for detecting opposite traversal
        # through the same edge.
        self.edge_distance = float(
            edge_distance
        )

        # Number of future path points inspected.
        self.prediction_horizon = int(
            prediction_horizon
        )

    # =========================================================
    # MAIN DETECTION ENTRY POINT
    # =========================================================

    def detect_conflicts(
        self,
        own_state,
        peer_states
    ):

        """
        Detect conflicts between one AMR and its known peers.

        Parameters
        ----------
        own_state:
            RobotState representing the local AMR.

        peer_states:
            Dictionary:
                robot_id -> RobotState

        Returns
        -------
        list[dict]

        Each conflict dictionary contains information such as:

            type
            robot_a
            robot_b
            distance
            path_step
            severity
        """

        conflicts = []

        if own_state is None:
            return conflicts

        if not peer_states:
            return conflicts

        for peer_id, peer_state in (
            peer_states.items()
        ):

            if peer_state is None:
                continue

            if (
                peer_state.robot_id
                == own_state.robot_id
            ):
                continue

            # -------------------------------------------------
            # 1. CURRENT PHYSICAL PROXIMITY
            # -------------------------------------------------

            proximity_conflict = (
                self.detect_proximity_conflict(
                    own_state,
                    peer_state
                )
            )

            if proximity_conflict:

                conflicts.append(
                    proximity_conflict
                )

            # -------------------------------------------------
            # 2. FUTURE VERTEX CONFLICT
            # -------------------------------------------------

            vertex_conflict = (
                self.detect_vertex_conflict(
                    own_state,
                    peer_state
                )
            )

            if vertex_conflict:

                conflicts.append(
                    vertex_conflict
                )

            # -------------------------------------------------
            # 3. FUTURE EDGE CONFLICT
            # -------------------------------------------------

            edge_conflict = (
                self.detect_edge_conflict(
                    own_state,
                    peer_state
                )
            )

            if edge_conflict:

                conflicts.append(
                    edge_conflict
                )

        return conflicts

    # =========================================================
    # PROXIMITY CONFLICT
    # =========================================================

    def detect_proximity_conflict(
        self,
        robot_a,
        robot_b
    ):

        """
        Detect whether two robots are currently closer than
        the configured safety distance.
        """

        distance = self.distance(
            robot_a.x,
            robot_a.y,
            robot_b.x,
            robot_b.y
        )

        if distance > self.safety_distance:
            return None

        severity = self.calculate_severity(
            distance,
            self.safety_distance
        )

        return {
            'type':
                'PROXIMITY',

            'robot_a':
                robot_a.robot_id,

            'robot_b':
                robot_b.robot_id,

            'distance':
                round(
                    distance,
                    3
                ),

            'threshold':
                self.safety_distance,

            'severity':
                severity,

            'position_a': [
                round(
                    robot_a.x,
                    3
                ),
                round(
                    robot_a.y,
                    3
                )
            ],

            'position_b': [
                round(
                    robot_b.x,
                    3
                ),
                round(
                    robot_b.y,
                    3
                )
            ]
        }

    # =========================================================
    # VERTEX CONFLICT
    # =========================================================

    def detect_vertex_conflict(
        self,
        robot_a,
        robot_b
    ):

        """
        Detect whether two robots intend to occupy approximately
        the same location at approximately the same future
        path step.

        This is the continuous-coordinate equivalent of a
        traditional MAPF vertex conflict.
        """

        path_a = self.prepare_path(
            robot_a
        )

        path_b = self.prepare_path(
            robot_b
        )

        if not path_a or not path_b:
            return None

        maximum_steps = min(
            len(path_a),
            len(path_b),
            self.prediction_horizon
        )

        for step in range(
            maximum_steps
        ):

            point_a = path_a[
                step
            ]

            point_b = path_b[
                step
            ]

            distance = self.distance(
                point_a[0],
                point_a[1],
                point_b[0],
                point_b[1]
            )

            if (
                distance
                <= self.vertex_distance
            ):

                return {
                    'type':
                        'VERTEX',

                    'robot_a':
                        robot_a.robot_id,

                    'robot_b':
                        robot_b.robot_id,

                    'path_step':
                        step,

                    'distance':
                        round(
                            distance,
                            3
                        ),

                    'threshold':
                        self.vertex_distance,

                    'severity':
                        self.calculate_severity(
                            distance,
                            self.vertex_distance
                        ),

                    'position_a': [
                        round(
                            point_a[0],
                            3
                        ),
                        round(
                            point_a[1],
                            3
                        )
                    ],

                    'position_b': [
                        round(
                            point_b[0],
                            3
                        ),
                        round(
                            point_b[1],
                            3
                        )
                    ]
                }

        return None

    # =========================================================
    # EDGE CONFLICT
    # =========================================================

    def detect_edge_conflict(
        self,
        robot_a,
        robot_b
    ):

        """
        Detect whether two robots intend to traverse the same
        path segment in opposite directions.

        Example:

            AMR-01:
                A -> B

            AMR-02:
                B -> A

        This represents a classical MAPF edge conflict.
        """

        path_a = self.prepare_path(
            robot_a
        )

        path_b = self.prepare_path(
            robot_b
        )

        if (
            len(path_a) < 2
            or len(path_b) < 2
        ):

            return None

        maximum_steps = min(
            len(path_a) - 1,
            len(path_b) - 1,
            self.prediction_horizon
        )

        for step in range(
            maximum_steps
        ):

            a_current = path_a[
                step
            ]

            a_next = path_a[
                step + 1
            ]

            b_current = path_b[
                step
            ]

            b_next = path_b[
                step + 1
            ]

            # Robot A's current point should approximately match
            # Robot B's next point.
            first_distance = self.distance(
                a_current[0],
                a_current[1],
                b_next[0],
                b_next[1]
            )

            # Robot A's next point should approximately match
            # Robot B's current point.
            second_distance = self.distance(
                a_next[0],
                a_next[1],
                b_current[0],
                b_current[1]
            )

            if (
                first_distance
                <= self.edge_distance
                and second_distance
                <= self.edge_distance
            ):

                average_distance = (
                    first_distance
                    + second_distance
                ) / 2.0

                return {
                    'type':
                        'EDGE',

                    'robot_a':
                        robot_a.robot_id,

                    'robot_b':
                        robot_b.robot_id,

                    'path_step':
                        step,

                    'distance':
                        round(
                            average_distance,
                            3
                        ),

                    'threshold':
                        self.edge_distance,

                    'severity':
                        self.calculate_severity(
                            average_distance,
                            self.edge_distance
                        ),

                    'edge_a': {
                        'from': [
                            round(
                                a_current[0],
                                3
                            ),
                            round(
                                a_current[1],
                                3
                            )
                        ],

                        'to': [
                            round(
                                a_next[0],
                                3
                            ),
                            round(
                                a_next[1],
                                3
                            )
                        ]
                    },

                    'edge_b': {
                        'from': [
                            round(
                                b_current[0],
                                3
                            ),
                            round(
                                b_current[1],
                                3
                            )
                        ],

                        'to': [
                            round(
                                b_next[0],
                                3
                            ),
                            round(
                                b_next[1],
                                3
                            )
                        ]
                    }
                }

        return None

    # =========================================================
    # PREPARE ROBOT PATH
    # =========================================================

    def prepare_path(
        self,
        robot_state
    ):

        """
        Build a clean path representation.

        The robot's current position is always inserted first.

        The remaining trajectory comes from the navigation
        intent published by robot_controller.py.
        """

        path = [
            (
                float(
                    robot_state.x
                ),
                float(
                    robot_state.y
                )
            )
        ]

        planned_path = getattr(
            robot_state,
            'planned_path',
            []
        )

        if not isinstance(
            planned_path,
            list
        ):

            return path

        for point in planned_path:

            if (
                not isinstance(
                    point,
                    (list, tuple)
                )
                or len(point) < 2
            ):

                continue

            try:

                x = float(
                    point[0]
                )

                y = float(
                    point[1]
                )

            except (
                TypeError,
                ValueError
            ):

                continue

            # Avoid adding the current position twice.
            if path:

                previous = path[
                    -1
                ]

                if (
                    self.distance(
                        previous[0],
                        previous[1],
                        x,
                        y
                    )
                    < 0.01
                ):

                    continue

            path.append(
                (
                    x,
                    y
                )
            )

        return path

    # =========================================================
    # CONFLICT BETWEEN TWO PATHS
    # =========================================================

    def paths_conflict(
        self,
        robot_a,
        robot_b
    ):

        """
        Convenience function.

        Returns True if any supported conflict is detected.
        """

        if self.detect_proximity_conflict(
            robot_a,
            robot_b
        ):

            return True

        if self.detect_vertex_conflict(
            robot_a,
            robot_b
        ):

            return True

        if self.detect_edge_conflict(
            robot_a,
            robot_b
        ):

            return True

        return False

    # =========================================================
    # FIND MOST SEVERE CONFLICT
    # =========================================================

    @staticmethod
    def most_severe_conflict(
        conflicts
    ):

        if not conflicts:
            return None

        return max(
            conflicts,
            key=lambda conflict:
                float(
                    conflict.get(
                        'severity',
                        0.0
                    )
                )
        )

    # =========================================================
    # SEVERITY
    # =========================================================

    @staticmethod
    def calculate_severity(
        distance,
        threshold
    ):

        """
        Convert separation into a simple normalized risk value.

        1.0 = extremely close
        0.0 = at or beyond threshold
        """

        if threshold <= 0.0:
            return 0.0

        severity = (
            1.0
            - (
                distance
                / threshold
            )
        )

        return round(
            max(
                0.0,
                min(
                    1.0,
                    severity
                )
            ),
            3
        )

    # =========================================================
    # EUCLIDEAN DISTANCE
    # =========================================================

    @staticmethod
    def distance(
        x1,
        y1,
        x2,
        y2
    ):

        return math.hypot(
            x2 - x1,
            y2 - y1
        )
        