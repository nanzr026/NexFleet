"""LaCAM + PIBT multi-agent safety planner adapter.

STATUS: ADAPTER / PENDING FULL INTEGRATION

This module provides the interface for LaCAM (Lazy Collision-Avoidance
with Minimal-effort) and PIBT (Priority Inheritance with Backtracking)
algorithms for high-risk multi-agent path planning.

Architecture:
    Normal: A*
    Medium risk: GNN + local negotiation
    High risk / unresolved: LaCAM + PIBT (this module)

Current implementation: DETERMINISTIC A*-BASED FALLBACK
    The full LaCAM+PIBT algorithm will be integrated from the
    existing external implementation under Ubuntu/ROS2.

Do NOT fake LaCAM success events. This adapter is honest about its status.
"""
from __future__ import annotations
from enum import Enum
from typing import Dict, List, Optional, Set, Tuple

from algorithms.astar.astar import astar_with_reservation


class LaCaMStatus(str, Enum):
    AVAILABLE = "AVAILABLE"            # Full LaCAM implementation loaded
    PENDING_INTEGRATION = "PENDING_INTEGRATION"  # Adapter only
    FALLBACK_ACTIVE = "FALLBACK_ACTIVE"          # Using A* reservation fallback


INTEGRATION_STATUS = LaCaMStatus.PENDING_INTEGRATION


class LaCaMPIBTAdapter:
    """
    Adapter for LaCAM + PIBT multi-agent safety planner.

    Current status: PENDING_INTEGRATION
    Using deterministic A*-with-reservation fallback.

    When the external LaCAM+PIBT implementation is integrated,
    replace _solve_with_fallback() with _solve_with_lacam().
    """

    def __init__(self):
        self.status = INTEGRATION_STATUS
        self._plans_computed = 0
        self._fallback_activations = 0

    def solve(
        self,
        starts: List[Tuple[int, int]],
        goals: List[Tuple[int, int]],
        robot_ids: List[str],
        passable_fn,
        neighbors_fn,
        timeout_ms: float = 500.0,
    ) -> Dict[str, Optional[List[Tuple[int, int]]]]:
        """
        Plan conflict-free paths for all robots.

        Returns dict: {robot_id: path or None}

        Status: PENDING_INTEGRATION (using A*-reservation fallback)
        """
        self._plans_computed += 1

        if self.status == LaCaMStatus.AVAILABLE:
            # Full LaCAM+PIBT would go here
            return self._solve_with_lacam(starts, goals, robot_ids, passable_fn, neighbors_fn)
        else:
            self._fallback_activations += 1
            return self._solve_with_fallback(
                starts, goals, robot_ids, passable_fn, neighbors_fn
            )

    def _solve_with_fallback(
        self,
        starts: List[Tuple[int, int]],
        goals: List[Tuple[int, int]],
        robot_ids: List[str],
        passable_fn,
        neighbors_fn,
    ) -> Dict[str, Optional[List[Tuple[int, int]]]]:
        """
        Deterministic reservation-based A* fallback.
        Priority: lower index = higher priority.
        """
        results: Dict[str, Optional[List[Tuple[int, int]]]] = {}
        reserved_cells: Set[Tuple[int, int]] = set()

        for robot_id, start, goal in zip(robot_ids, starts, goals):
            path = astar_with_reservation(
                start=start,
                goal=goal,
                passable_fn=passable_fn,
                neighbors_fn=neighbors_fn,
                reserved_cells=reserved_cells,
            )
            results[robot_id] = path
            if path:
                # Reserve intermediate cells for lower-priority robots
                reserved_cells.update(path[1:-1])

        return results

    def _solve_with_lacam(
        self,
        starts: List[Tuple[int, int]],
        goals: List[Tuple[int, int]],
        robot_ids: List[str],
        passable_fn,
        neighbors_fn,
    ) -> Dict[str, Optional[List[Tuple[int, int]]]]:
        """
        Placeholder for full LaCAM+PIBT integration.
        Integrate external LaCAM implementation here.
        """
        raise NotImplementedError(
            "Full LaCAM+PIBT not yet integrated. "
            "Provide LaCAM source and call _solve_with_lacam."
        )

    def get_status(self) -> dict:
        return {
            "lacam_pibt_status": self.status.value,
            "plans_computed": self._plans_computed,
            "fallback_activations": self._fallback_activations,
            "note": (
                "LaCAM+PIBT full implementation pending integration from external source. "
                "Currently using A*-with-reservation fallback for high-risk scenarios."
            ),
        }
