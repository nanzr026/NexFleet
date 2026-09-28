"""
EdgeFleet-AI
CBS-Style Decentralized Safety Layer

This module converts detected robot-robot conflicts into deterministic
safety decisions.

The purpose of this layer is to act as a safety shield between the
coordination/planning layer and the physical robot controller.

Current supported decisions:
    PROCEED
    YIELD

Later stages can extend this with:
    WAIT
    RESUME
    REPLAN
    REROUTE

This is intentionally decentralized. Every AMR can independently
calculate the same winner/yielder using shared RobotState information.
"""


class CBSSafety:
    """
    Deterministic decentralized CBS-style safety decision module.

    The module does not directly control robot motion.

    It determines which robot should proceed and which robot should
    yield when a conflict has already been detected.
    """

    def __init__(self):
        # Higher value = more important conflict.
        self.conflict_priority = {
            'PROXIMITY': 3,
            'EDGE': 2,
            'VERTEX': 1,
        }

    # ==============================================================
    # PUBLIC API
    # ==============================================================

    def resolve_conflict(
        self,
        own_state,
        peer_state,
        conflict,
    ):
        """
        Resolve one conflict between two robots.

        Returns a dictionary describing the deterministic safety
        decision.

        Example:

        {
            "conflict_type": "VERTEX",
            "winner": "amr_01",
            "yielder": "amr_02",
            "own_action": "PROCEED",
            "peer_action": "YIELD",
            "reason": "..."
        }
        """

        if own_state is None or peer_state is None:
            return None

        if conflict is None:
            return None

        winner, yielder, reason = self.select_priority_robot(
            own_state,
            peer_state,
        )

        if own_state.robot_id == winner:
            own_action = 'PROCEED'
            peer_action = 'YIELD'
        else:
            own_action = 'YIELD'
            peer_action = 'PROCEED'

        return {
            'conflict_type': conflict.get('type', 'UNKNOWN'),
            'winner': winner,
            'yielder': yielder,
            'own_robot': own_state.robot_id,
            'peer_robot': peer_state.robot_id,
            'own_action': own_action,
            'peer_action': peer_action,
            'reason': reason,
            'conflict': conflict,
        }

    # ==============================================================
    # PRIORITY SELECTION
    # ==============================================================

    def select_priority_robot(
        self,
        robot_a,
        robot_b,
    ):
        """
        Select which robot receives right-of-way.

        Priority order:

        1. Explicit task priority
        2. Lower battery robot
        3. Robot closer to its goal
        4. Deterministic robot ID tie-breaker

        Both robots have the same shared information, so both should
        independently reach the same decision.
        """

        # ----------------------------------------------------------
        # RULE 1
        # Explicit task priority
        # ----------------------------------------------------------

        priority_a = self.safe_number(
            getattr(robot_a, 'priority', 0),
            0,
        )

        priority_b = self.safe_number(
            getattr(robot_b, 'priority', 0),
            0,
        )

        if priority_a > priority_b:
            return (
                robot_a.robot_id,
                robot_b.robot_id,
                'Higher task priority',
            )

        if priority_b > priority_a:
            return (
                robot_b.robot_id,
                robot_a.robot_id,
                'Higher task priority',
            )

        # ----------------------------------------------------------
        # RULE 2
        # Lower battery gets right-of-way.
        #
        # Reason:
        # A lower-battery AMR should avoid unnecessary waiting when
        # all higher-level task priorities are equal.
        # ----------------------------------------------------------

        battery_a = self.safe_number(
            getattr(robot_a, 'battery', 100.0),
            100.0,
        )

        battery_b = self.safe_number(
            getattr(robot_b, 'battery', 100.0),
            100.0,
        )

        battery_difference = abs(
            battery_a - battery_b
        )

        # Ignore tiny differences to avoid unstable decisions.
        if battery_difference >= 5.0:

            if battery_a < battery_b:
                return (
                    robot_a.robot_id,
                    robot_b.robot_id,
                    'Lower battery receives right-of-way',
                )

            return (
                robot_b.robot_id,
                robot_a.robot_id,
                'Lower battery receives right-of-way',
            )

        # ----------------------------------------------------------
        # RULE 3
        # Robot closer to its current goal gets right-of-way.
        # ----------------------------------------------------------

        distance_a = self.distance_to_goal(
            robot_a
        )

        distance_b = self.distance_to_goal(
            robot_b
        )

        if (
            distance_a is not None
            and distance_b is not None
        ):

            distance_difference = abs(
                distance_a - distance_b
            )

            # Small differences are ignored.
            if distance_difference >= 0.25:

                if distance_a < distance_b:
                    return (
                        robot_a.robot_id,
                        robot_b.robot_id,
                        'Closer to goal',
                    )

                return (
                    robot_b.robot_id,
                    robot_a.robot_id,
                    'Closer to goal',
                )

        # ----------------------------------------------------------
        # RULE 4
        # Deterministic robot ID tie-breaker.
        #
        # This is essential in decentralized operation.
        #
        # Example:
        #
        # amr_01 < amr_02
        #
        # Therefore both AMRs independently choose amr_01.
        # ----------------------------------------------------------

        if robot_a.robot_id <= robot_b.robot_id:
            return (
                robot_a.robot_id,
                robot_b.robot_id,
                'Deterministic robot ID tie-breaker',
            )

        return (
            robot_b.robot_id,
            robot_a.robot_id,
            'Deterministic robot ID tie-breaker',
        )

    # ==============================================================
    # MULTIPLE CONFLICTS
    # ==============================================================

    def resolve_multiple_conflicts(
        self,
        own_state,
        peer_states,
        conflicts,
    ):
        """
        Resolve multiple conflicts involving the local robot.

        The safest local action is returned.

        If the local robot must yield to ANY peer, the final local
        action becomes YIELD.

        Otherwise the robot may PROCEED.
        """

        if not conflicts:
            return {
                'own_robot': own_state.robot_id,
                'final_action': 'PROCEED',
                'decisions': [],
            }

        decisions = []

        for conflict in conflicts:

            robot_a = conflict.get('robot_a')
            robot_b = conflict.get('robot_b')

            peer_id = None

            if robot_a == own_state.robot_id:
                peer_id = robot_b

            elif robot_b == own_state.robot_id:
                peer_id = robot_a

            if peer_id is None:
                continue

            peer_state = peer_states.get(
                peer_id
            )

            if peer_state is None:
                continue

            decision = self.resolve_conflict(
                own_state,
                peer_state,
                conflict,
            )

            if decision is not None:
                decisions.append(decision)

        final_action = 'PROCEED'

        for decision in decisions:

            if decision['own_action'] == 'YIELD':
                final_action = 'YIELD'
                break

        return {
            'own_robot': own_state.robot_id,
            'final_action': final_action,
            'decisions': decisions,
        }

    # ==============================================================
    # GOAL DISTANCE
    # ==============================================================

    def distance_to_goal(
        self,
        robot_state,
    ):
        """
        Calculate straight-line distance between the robot and its
        current goal.

        Returns None if the robot has no valid goal.
        """

        goal_x = getattr(
            robot_state,
            'goal_x',
            None,
        )

        goal_y = getattr(
            robot_state,
            'goal_y',
            None,
        )

        if goal_x is None or goal_y is None:
            return None

        x = self.safe_number(
            getattr(robot_state, 'x', 0.0),
            0.0,
        )

        y = self.safe_number(
            getattr(robot_state, 'y', 0.0),
            0.0,
        )

        goal_x = self.safe_number(
            goal_x,
            None,
        )

        goal_y = self.safe_number(
            goal_y,
            None,
        )

        if goal_x is None or goal_y is None:
            return None

        dx = goal_x - x
        dy = goal_y - y

        return (
            dx * dx + dy * dy
        ) ** 0.5

    # ==============================================================
    # CONFLICT IMPORTANCE
    # ==============================================================

    def get_conflict_priority(
        self,
        conflict,
    ):
        """
        Return the safety priority of a conflict type.
        """

        if conflict is None:
            return 0

        conflict_type = conflict.get(
            'type',
            'UNKNOWN',
        )

        return self.conflict_priority.get(
            conflict_type,
            0,
        )

    def get_most_critical_conflict(
        self,
        conflicts,
    ):
        """
        Select the most safety-critical conflict.

        Primary:
            conflict type priority

        Secondary:
            reported severity
        """

        if not conflicts:
            return None

        return max(
            conflicts,
            key=lambda conflict: (
                self.get_conflict_priority(
                    conflict
                ),
                self.safe_number(
                    conflict.get(
                        'severity',
                        0.0,
                    ),
                    0.0,
                ),
            ),
        )

    # ==============================================================
    # UTILITY
    # ==============================================================

    @staticmethod
    def safe_number(
        value,
        default=0.0,
    ):
        """
        Convert a value to float safely.
        """

        try:
            if value is None:
                return default

            return float(value)

        except (
            TypeError,
            ValueError,
        ):
            return default