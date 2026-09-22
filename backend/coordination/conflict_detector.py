"""Deterministic conflict detection for NEXFLEET safety layer."""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Set, Tuple

from backend.simulation.amr_agent import AMRState


class ConflictType(str, Enum):
    VERTEX = "VERTEX"     # Two robots at same cell
    EDGE = "EDGE"         # Two robots swap positions
    SWAP = "SWAP"         # Same as edge but at next tick
    OCCUPANCY = "OCCUPANCY"  # Intended move into occupied cell


@dataclass
class Conflict:
    conflict_type: ConflictType
    robot_ids: List[str]
    position: Tuple[int, int]
    tick: int
    description: str

    def to_dict(self) -> dict:
        return {
            "type": self.conflict_type.value,
            "robots": self.robot_ids,
            "position": list(self.position),
            "tick": self.tick,
            "description": self.description,
        }


class ConflictDetector:
    """Detects vertex, edge, swap, and occupancy conflicts."""

    def detect(
        self,
        states: List[AMRState],
        intended_moves: Dict[str, Optional[Tuple[int, int]]],
        tick: int,
    ) -> List[Conflict]:
        conflicts = []
        active = [s for s in states if s.status != "FAILED"]

        # Current positions
        pos_to_robots: Dict[Tuple[int, int], List[str]] = {}
        for s in active:
            p = (s.row, s.col)
            pos_to_robots.setdefault(p, []).append(s.robot_id)

        # Vertex conflicts (already same cell)
        for pos, robots in pos_to_robots.items():
            if len(robots) > 1:
                conflicts.append(Conflict(
                    ConflictType.VERTEX, robots, pos, tick,
                    f"Vertex conflict: {robots} at {pos}"
                ))

        # Edge/swap conflicts from intended moves
        robot_pos = {s.robot_id: (s.row, s.col) for s in active}
        intent_pos = {
            rid: move for rid, move in intended_moves.items()
            if move is not None
        }

        # Occupancy conflicts: two robots intend to move to same cell
        dest_to_robots: Dict[Tuple[int, int], List[str]] = {}
        for rid, dest in intent_pos.items():
            dest_to_robots.setdefault(dest, []).append(rid)
        for dest, robots in dest_to_robots.items():
            if len(robots) > 1:
                conflicts.append(Conflict(
                    ConflictType.OCCUPANCY, robots, dest, tick,
                    f"Occupancy conflict: {robots} all intend to move to {dest}"
                ))

        # Edge/swap: robot A moves from X to Y, robot B moves from Y to X
        rids = list(intent_pos.keys())
        for i in range(len(rids)):
            for j in range(i + 1, len(rids)):
                ri, rj = rids[i], rids[j]
                pi, pj = robot_pos.get(ri), robot_pos.get(rj)
                di, dj = intent_pos[ri], intent_pos[rj]
                if pi and pj and di and dj:
                    if di == pj and dj == pi:
                        conflicts.append(Conflict(
                            ConflictType.SWAP, [ri, rj],
                            di, tick,
                            f"Swap conflict: {ri} and {rj} swapping positions"
                        ))

        return conflicts

    def check_reservation(
        self,
        robot_id: str,
        intended_pos: Tuple[int, int],
        reservation_table: Dict[Tuple[int, int], str],
    ) -> bool:
        """
        Check if a cell is already reserved by another robot.
        Returns True if safe (not reserved by another).
        """
        owner = reservation_table.get(intended_pos)
        return owner is None or owner == robot_id


class ReservationTable:
    """Simple single-tick cell reservation table."""

    def __init__(self):
        self._table: Dict[Tuple[int, int], str] = {}  # pos -> robot_id

    def reserve(self, pos: Tuple[int, int], robot_id: str) -> bool:
        """Reserve a cell. Returns True if successful."""
        existing = self._table.get(pos)
        if existing is None or existing == robot_id:
            self._table[pos] = robot_id
            return True
        return False

    def release(self, robot_id: str):
        """Release all reservations held by a robot."""
        self._table = {p: r for p, r in self._table.items() if r != robot_id}

    def clear(self):
        self._table.clear()

    def is_reserved_by_other(self, pos: Tuple[int, int], robot_id: str) -> bool:
        existing = self._table.get(pos)
        return existing is not None and existing != robot_id

    def get_table(self) -> Dict[Tuple[int, int], str]:
        return dict(self._table)
