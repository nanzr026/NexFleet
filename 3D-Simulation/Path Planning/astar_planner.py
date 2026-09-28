#!/usr/bin/env python3

import heapq
import math


class AStarPlanner:

    def __init__(self, warehouse_map):

        self.map = warehouse_map

        # =====================================================
        # 8-DIRECTION GRID MOVEMENT
        # =====================================================

        self.movements = [
            (1, 0, 1.0),
            (-1, 0, 1.0),
            (0, 1, 1.0),
            (0, -1, 1.0),

            (1, 1, math.sqrt(2)),
            (1, -1, math.sqrt(2)),
            (-1, 1, math.sqrt(2)),
            (-1, -1, math.sqrt(2)),
        ]

    # =========================================================
    # HEURISTIC
    # =========================================================

    @staticmethod
    def heuristic(a, b):

        dx = a[0] - b[0]
        dy = a[1] - b[1]

        return math.hypot(
            dx,
            dy
        )

    # =========================================================
    # PLAN
    # =========================================================

    def plan(self, start_world, goal_world):

        start = self.map.world_to_grid(
            start_world[0],
            start_world[1]
        )

        goal = self.map.world_to_grid(
            goal_world[0],
            goal_world[1]
        )

        print(
            f'A* START world={start_world} '
            f'grid={start}'
        )

        print(
            f'A* GOAL  world={goal_world} '
            f'grid={goal}'
        )

        # =====================================================
        # VALIDATION
        # =====================================================

        if not self.map.is_valid_grid(*start):

            raise ValueError(
                f'Start position {start_world} '
                f'is outside the warehouse map'
            )

        if not self.map.is_valid_grid(*goal):

            raise ValueError(
                f'Goal position {goal_world} '
                f'is outside the warehouse map'
            )

        if self.map.is_occupied_grid(*start):

            raise ValueError(
                f'Start position {start_world} '
                f'is inside an obstacle'
            )

        if self.map.is_occupied_grid(*goal):

            raise ValueError(
                f'Goal position {goal_world} '
                f'is inside an obstacle'
            )

        # =====================================================
        # A* DATA STRUCTURES
        # =====================================================

        open_heap = []

        counter = 0

        start_h = self.heuristic(
            start,
            goal
        )

        heapq.heappush(
            open_heap,
            (
                start_h,
                counter,
                start
            )
        )

        came_from = {}

        g_score = {
            start: 0.0
        }

        closed = set()

        # =====================================================
        # A* SEARCH
        # =====================================================

        while open_heap:

            _, _, current = heapq.heappop(
                open_heap
            )

            if current in closed:
                continue

            # -------------------------------------------------
            # GOAL FOUND
            # -------------------------------------------------

            if current == goal:

                grid_path = self.reconstruct_path(
                    came_from,
                    current
                )

                world_path = [

                    self.map.grid_to_world(
                        gx,
                        gy
                    )

                    for gx, gy in grid_path
                ]

                # ---------------------------------------------
                # Preserve exact requested start and goal.
                #
                # The grid path normally starts/ends at the
                # center of a 0.20 m occupancy cell.
                #
                # For robot navigation we want the actual
                # requested coordinates.
                # ---------------------------------------------

                if world_path:

                    world_path[0] = (
                        float(start_world[0]),
                        float(start_world[1])
                    )

                    world_path[-1] = (
                        float(goal_world[0]),
                        float(goal_world[1])
                    )

                # ---------------------------------------------
                # Smooth the raw A* path.
                # ---------------------------------------------

                smoothed_path = self.smooth_path(
                    world_path
                )

                # Ensure exact endpoints again after smoothing.

                if smoothed_path:

                    smoothed_path[0] = (
                        float(start_world[0]),
                        float(start_world[1])
                    )

                    smoothed_path[-1] = (
                        float(goal_world[0]),
                        float(goal_world[1])
                    )

                return smoothed_path

            closed.add(
                current
            )

            cx, cy = current

            # =================================================
            # CHECK NEIGHBOURS
            # =================================================

            for dx, dy, move_cost in self.movements:

                nx = cx + dx
                ny = cy + dy

                neighbour = (
                    nx,
                    ny
                )

                # ---------------------------------------------
                # MAP BOUNDARY
                # ---------------------------------------------

                if not self.map.is_valid_grid(
                    nx,
                    ny
                ):
                    continue

                # ---------------------------------------------
                # OBSTACLE
                # ---------------------------------------------

                if self.map.is_occupied_grid(
                    nx,
                    ny
                ):
                    continue

                # ---------------------------------------------
                # PREVENT DIAGONAL CORNER CUTTING
                # ---------------------------------------------

                if dx != 0 and dy != 0:

                    horizontal_blocked = (
                        self.map.is_occupied_grid(
                            cx + dx,
                            cy
                        )
                    )

                    vertical_blocked = (
                        self.map.is_occupied_grid(
                            cx,
                            cy + dy
                        )
                    )

                    if (
                        horizontal_blocked
                        or vertical_blocked
                    ):
                        continue

                # ---------------------------------------------
                # COST
                # ---------------------------------------------

                tentative_g = (
                    g_score[current]
                    + move_cost
                )

                previous_g = g_score.get(
                    neighbour,
                    float('inf')
                )

                if tentative_g >= previous_g:
                    continue

                # Better route found.

                came_from[neighbour] = current

                g_score[neighbour] = tentative_g

                f_score = (
                    tentative_g
                    + self.heuristic(
                        neighbour,
                        goal
                    )
                )

                counter += 1

                heapq.heappush(
                    open_heap,
                    (
                        f_score,
                        counter,
                        neighbour
                    )
                )

        # =====================================================
        # NO PATH
        # =====================================================

        return None

    # =========================================================
    # RECONSTRUCT RAW A* PATH
    # =========================================================

    @staticmethod
    def reconstruct_path(
        came_from,
        current
    ):

        path = [
            current
        ]

        while current in came_from:

            current = came_from[
                current
            ]

            path.append(
                current
            )

        path.reverse()

        return path

    # =========================================================
    # LINE-OF-SIGHT COLLISION CHECK
    # =========================================================

    def has_line_of_sight(
        self,
        point_a,
        point_b
    ):

        gx1, gy1 = self.map.world_to_grid(
            point_a[0],
            point_a[1]
        )

        gx2, gy2 = self.map.world_to_grid(
            point_b[0],
            point_b[1]
        )

        dx = gx2 - gx1
        dy = gy2 - gy1

        steps = max(
            abs(dx),
            abs(dy)
        )

        if steps == 0:
            return True

        # -----------------------------------------------------
        # Use multiple samples per grid cell.
        #
        # This makes the check conservative near obstacle
        # boundaries.
        # -----------------------------------------------------

        samples = max(
            1,
            steps * 3
        )

        previous_gx = gx1
        previous_gy = gy1

        for index in range(
            samples + 1
        ):

            ratio = (
                index
                / samples
            )

            gx = int(
                round(
                    gx1
                    + dx * ratio
                )
            )

            gy = int(
                round(
                    gy1
                    + dy * ratio
                )
            )

            # ---------------------------------------------
            # MAP BOUNDARY
            # ---------------------------------------------

            if not self.map.is_valid_grid(
                gx,
                gy
            ):
                return False

            # ---------------------------------------------
            # OCCUPIED CELL
            # ---------------------------------------------

            if self.map.is_occupied_grid(
                gx,
                gy
            ):
                return False

            # ---------------------------------------------
            # DIAGONAL CORNER PROTECTION
            # ---------------------------------------------

            step_x = (
                gx
                - previous_gx
            )

            step_y = (
                gy
                - previous_gy
            )

            if (
                step_x != 0
                and step_y != 0
            ):

                check_x = (
                    previous_gx
                    + step_x
                )

                check_y = (
                    previous_gy
                    + step_y
                )

                if (
                    self.map.is_occupied_grid(
                        check_x,
                        previous_gy
                    )
                    or
                    self.map.is_occupied_grid(
                        previous_gx,
                        check_y
                    )
                ):
                    return False

            previous_gx = gx
            previous_gy = gy

        return True

    # =========================================================
    # PATH SMOOTHING
    # =========================================================

    def smooth_path(self, path):

        if path is None:
            return None

        if len(path) <= 2:
            return path

        smoothed = [
            path[0]
        ]

        anchor_index = 0

        # =====================================================
        # GREEDY LINE-OF-SIGHT SMOOTHING
        #
        # From each waypoint, search backwards from the goal
        # for the furthest waypoint that can be reached by a
        # collision-free straight segment.
        # =====================================================

        while anchor_index < len(path) - 1:

            furthest_visible = (
                anchor_index + 1
            )

            for candidate_index in range(
                len(path) - 1,
                anchor_index,
                -1
            ):

                if self.has_line_of_sight(
                    path[anchor_index],
                    path[candidate_index]
                ):

                    furthest_visible = (
                        candidate_index
                    )

                    break

            # Safety protection against an infinite loop.

            if furthest_visible <= anchor_index:

                furthest_visible = (
                    anchor_index + 1
                )

            smoothed.append(
                path[furthest_visible]
            )

            anchor_index = (
                furthest_visible
            )

        # =====================================================
        # REMOVE ACCIDENTAL DUPLICATES
        # =====================================================

        cleaned = []

        for point in smoothed:

            if not cleaned:

                cleaned.append(
                    point
                )

                continue

            previous = cleaned[-1]

            distance = math.hypot(
                point[0] - previous[0],
                point[1] - previous[1]
            )

            if distance > 0.01:

                cleaned.append(
                    point
                )

        return cleaned