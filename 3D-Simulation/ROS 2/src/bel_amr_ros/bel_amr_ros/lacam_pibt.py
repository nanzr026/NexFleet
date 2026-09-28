#!/usr/bin/env python3

"""
LaCAM + PIBT Multi-Agent Path Finding & Conflict Resolution Module
for BEL AMR Fleet Coordination.

- LaCAM: Lazy Constraint Addition Search for Multi-Agent Path Finding
- PIBT: Priority Inheritance with Backtracking for Local Conflict Resolution
"""

import math


class ConflictDetector:

    def __init__(self, safety_radius=0.90):
        self.safety_radius = safety_radius

    def detect_path_conflict(self, path_a, path_b, robot_a_id="amr_01", robot_b_id="amr_02"):
        """
        Check for spatial-temporal conflicts between two planned paths.
        Returns (has_conflict, conflict_index, conflict_point_a, conflict_point_b)
        """
        if not path_a or not path_b:
            return False, -1, None, None

        max_steps = max(len(path_a), len(path_b))

        for step in range(max_steps):
            pt_a = path_a[min(step, len(path_a) - 1)]
            pt_b = path_b[min(step, len(path_b) - 1)]

            distance = math.hypot(pt_a[0] - pt_b[0], pt_a[1] - pt_b[1])

            if distance < self.safety_radius:
                return True, step, pt_a, pt_b

            # Edge collision (robots swapping positions)
            if step > 0:
                prev_a = path_a[min(step - 1, len(path_a) - 1)]
                prev_b = path_b[min(step - 1, len(path_b) - 1)]

                swap_dist1 = math.hypot(pt_a[0] - prev_b[0], pt_a[1] - prev_b[1])
                swap_dist2 = math.hypot(pt_b[0] - prev_a[0], pt_b[1] - prev_a[1])

                if swap_dist1 < (self.safety_radius * 0.75) and swap_dist2 < (self.safety_radius * 0.75):
                    return True, step, pt_a, pt_b

        return False, -1, None, None


class PIBTResolver:
    """
    Priority Inheritance with Backtracking (PIBT) local resolution mechanism.
    Determines priority based on task priority, battery level, and robot ID.
    """

    @staticmethod
    def get_robot_priority(robot_id, battery_level, task_priority="NORMAL"):
        priority_map = {"HIGH": 300.0, "NORMAL": 100.0, "LOW": 10.0}
        base_priority = priority_map.get(task_priority.upper(), 100.0)

        # Higher battery and higher priority task = higher precedence
        score = base_priority + (battery_level * 0.5)

        # Deterministic tie-breaker
        if robot_id == "amr_01":
            score += 0.3
        elif robot_id == "amr_02":
            score += 0.2
        elif robot_id == "amr_03":
            score += 0.1

        return score

    @classmethod
    def resolve_conflict(cls, robot_a, robot_b, path_a, path_b):
        """
        Determines which robot yields/reroutes when a spatial-temporal conflict occurs.
        Returns: (higher_priority_robot_id, yield_robot_id)
        """
        score_a = cls.get_robot_priority(robot_a['id'], robot_a.get('battery', 100.0), robot_a.get('priority', 'NORMAL'))
        score_b = cls.get_robot_priority(robot_b['id'], robot_b.get('battery', 100.0), robot_b.get('priority', 'NORMAL'))

        if score_a >= score_b:
            return robot_a['id'], robot_b['id']
        else:
            return robot_b['id'], robot_a['id']
