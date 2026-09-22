"""Real A* pathfinding for the NEXFLEET warehouse grid."""
from __future__ import annotations
import heapq
import math
from typing import Callable, Dict, List, Optional, Set, Tuple


def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    """Manhattan distance heuristic."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(
    start: Tuple[int, int],
    goal: Tuple[int, int],
    passable_fn: Callable[[int, int], bool],
    neighbors_fn: Callable[[int, int], List[Tuple[int, int]]],
    reserved: Optional[Set[Tuple[int, int]]] = None,
) -> Optional[List[Tuple[int, int]]]:
    """
    A* search returning a list of (row, col) positions from start to goal
    (inclusive). Returns None if no path exists.

    Args:
        start: Starting position (row, col).
        goal: Goal position (row, col).
        passable_fn: Function returning True if a cell is passable.
        neighbors_fn: Function returning passable neighbors for a cell.
        reserved: Optional set of cells to treat as blocked (for reservation).
    """
    if start == goal:
        return [start]
    if not passable_fn(goal[0], goal[1]):
        return None

    blocked = reserved or set()

    open_set: List[Tuple[float, Tuple[int, int]]] = []
    heapq.heappush(open_set, (0.0, start))

    came_from: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {start: None}
    g_score: Dict[Tuple[int, int], float] = {start: 0.0}
    f_score: Dict[Tuple[int, int], float] = {start: heuristic(start, goal)}

    visited: Set[Tuple[int, int]] = set()

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return _reconstruct_path(came_from, current)

        if current in visited:
            continue
        visited.add(current)

        for neighbor in neighbors_fn(current[0], current[1]):
            if neighbor in visited:
                continue
            if neighbor in blocked and neighbor != goal:
                continue

            tentative_g = g_score[current] + 1.0
            if tentative_g < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic(neighbor, goal)
                f_score[neighbor] = f
                heapq.heappush(open_set, (f, neighbor))

    return None  # No path found


def _reconstruct_path(
    came_from: Dict[Tuple[int, int], Optional[Tuple[int, int]]],
    current: Tuple[int, int],
) -> List[Tuple[int, int]]:
    path = []
    while current is not None:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path


def astar_with_reservation(
    start: Tuple[int, int],
    goal: Tuple[int, int],
    passable_fn: Callable[[int, int], bool],
    neighbors_fn: Callable[[int, int], List[Tuple[int, int]]],
    reserved_cells: Optional[Set[Tuple[int, int]]] = None,
    max_iterations: int = 10000,
) -> Optional[List[Tuple[int, int]]]:
    """A* with reservation table support and iteration cap."""
    if start == goal:
        return [start]
    if not passable_fn(goal[0], goal[1]):
        return None

    blocked = reserved_cells or set()

    open_set: List[Tuple[float, int, Tuple[int, int]]] = []
    counter = 0
    heapq.heappush(open_set, (0.0, counter, start))

    came_from: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {start: None}
    g_score: Dict[Tuple[int, int], float] = {start: 0.0}
    visited: Set[Tuple[int, int]] = set()
    iterations = 0

    while open_set and iterations < max_iterations:
        iterations += 1
        _, _, current = heapq.heappop(open_set)

        if current == goal:
            return _reconstruct_path(came_from, current)

        if current in visited:
            continue
        visited.add(current)

        for neighbor in neighbors_fn(current[0], current[1]):
            if neighbor in visited:
                continue
            if neighbor in blocked and neighbor != goal:
                continue

            tentative_g = g_score[current] + 1.0
            if tentative_g < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic(neighbor, goal)
                counter += 1
                heapq.heappush(open_set, (f, counter, neighbor))

    return None
