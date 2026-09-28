"""Collision-aware pallet approach selection, independent of ROS."""

import math
import copy


def docking_distance(direction, pallet_yaw=0.0):
    """Support distances for the 0.85 m AMR and 1.22 x 0.8 m pallet."""
    angle = direction - pallet_yaw
    return 0.425 + 0.61 * abs(math.cos(angle)) + 0.40 * abs(math.sin(angle)) + 0.075


def choose_approach(planner, robot, pallet, pallet_yaw=0.0, departure_planner=None):
    """Choose the shortest reachable cardinal approach; never retreat to dock."""
    # Docking clearance uses the target's exact support geometry below.
    # Remove only its coarse inflated grid footprint, retaining other obstacles.
    clearance = copy.copy(planner)
    clearance.map = copy.copy(planner.map)
    clearance.map.pallets = {
        name: pose for name, pose in planner.map.pallets.items()
        if math.dist(pose[:2], pallet) > 0.05
    }

    def can_depart(angle, dock):
        if departure_planner is None:
            return True
        exit_point = (dock[0] + 1.4 * math.cos(angle),
                      dock[1] + 1.4 * math.sin(angle))
        return (clearance.has_line_of_sight(dock, exit_point) and
                not departure_planner.map.is_occupied_grid(
                    *departure_planner.map.world_to_grid(*exit_point)))

    bearing = math.atan2(robot[1] - pallet[1], robot[0] - pallet[0])
    distance = math.dist(robot, pallet)
    if (0.80 <= distance <= docking_distance(bearing, pallet_yaw) + 0.15
            and can_depart(bearing, robot)):
        return robot, [robot]
    candidates = []
    for index in range(4):
        angle = pallet_yaw + index * math.pi / 2
        radius = docking_distance(angle, pallet_yaw) + 0.40
        goal = (pallet[0] + radius * math.cos(angle),
                pallet[1] + radius * math.sin(angle))
        dock = (pallet[0] + (radius - 0.40) * math.cos(angle),
                pallet[1] + (radius - 0.40) * math.sin(angle))
        if not clearance.has_line_of_sight(goal, dock) or not can_depart(angle, dock):
            continue
        try:
            path = planner.plan(robot, goal)
        except ValueError:
            continue
        if path:
            length = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
            candidates.append((length, index, goal, path))
    if not candidates:
        raise ValueError('No safe reachable pallet approach')
    _, _, goal, path = min(candidates)
    return goal, path
