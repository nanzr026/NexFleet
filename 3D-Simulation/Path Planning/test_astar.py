#!/usr/bin/env python3

import math

from warehouse_map import WarehouseMap
from astar_planner import AStarPlanner


def path_length(path):

    if not path:
        return 0.0

    total = 0.0

    for i in range(
        1,
        len(path)
    ):

        x1, y1 = path[i - 1]
        x2, y2 = path[i]

        total += math.hypot(
            x2 - x1,
            y2 - y1
        )

    return total


def main():

    print()
    print(
        '========================================'
    )

    print(
        'BEL AMR - A* PATH PLANNING TEST'
    )

    print(
        '========================================'
    )

    warehouse = WarehouseMap()

    planner = AStarPlanner(
        warehouse
    )

    # =====================================================
    # AMR01 WAREHOUSE START
    # =====================================================

    start = (
        13.90,
        -10.60
    )

    # =====================================================
    # PRODUCT 01
    #
    # Actual pallet:
    # pallet_box_mobile
    # center = (4.4161, 14.6952)
    #
    # We intentionally do NOT drive into pallet center.
    # =====================================================

    product = (
        4.4161,
        14.6952
    )

    # Temporary docking / approach point.
    goal = (
        4.4161,
        13.40
    )

    print(
        f'AMR01 start: '
        f'({start[0]:.2f}, '
        f'{start[1]:.2f})'
    )

    print(
        f'PRODUCT_01: '
        f'({product[0]:.2f}, '
        f'{product[1]:.2f})'
    )

    print(
        f'Approach goal: '
        f'({goal[0]:.2f}, '
        f'{goal[1]:.2f})'
    )

    print(
        '========================================'
    )

    path = planner.plan(
        start,
        goal
    )

    if path is None:

        print()
        print(
            'A* FAILED: No path found.'
        )

        return

    print()
    print(
        'A* PATH FOUND'
    )

    print(
        '========================================'
    )

    print(
        f'Waypoints: {len(path)}'
    )

    print(
        f'Path length: '
        f'{path_length(path):.2f} m'
    )

    print(
        '========================================'
    )

    for index, waypoint in enumerate(
        path
    ):

        print(
            f'Waypoint {index:02d}: '
            f'('
            f'{waypoint[0]:.2f}, '
            f'{waypoint[1]:.2f}'
            f')'
        )

    print(
        '========================================'
    )


if __name__ == '__main__':

    main()