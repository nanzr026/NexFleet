#!/usr/bin/env python3

import math


class WarehouseMap:

    def __init__(self):

        # =====================================================
        # MAP CONFIGURATION
        # =====================================================

        self.resolution = 0.20

        self.min_x = -16.0
        self.max_x = 18.0

        self.min_y = -27.0
        self.max_y = 26.0

        self.width = int(
            math.ceil(
                (self.max_x - self.min_x)
                / self.resolution
            )
        )

        self.height = int(
            math.ceil(
                (self.max_y - self.min_y)
                / self.resolution
            )
        )

        # =====================================================
        # AMR SAFETY INFLATION
        # =====================================================

        # AMR base is approximately:
        # 0.85 m x 0.65 m
        #
        # 0.55 m inflation gives the planner clearance around
        # shelves instead of treating the AMR as a point.

        self.robot_inflation = 0.55

        # =====================================================
        # OCCUPANCY GRID
        #
        # False = FREE
        # True  = OCCUPIED
        # =====================================================

        self.grid = [
            [False for _ in range(self.height)]
            for _ in range(self.width)
        ]

        self.obstacles = []

        self.build_static_map()

    # =========================================================
    # WORLD -> GRID
    # =========================================================

    def world_to_grid(self, x, y):

        gx = int(
            math.floor(
                (x - self.min_x)
                / self.resolution
            )
        )

        gy = int(
            math.floor(
                (y - self.min_y)
                / self.resolution
            )
        )

        return gx, gy

    # =========================================================
    # GRID -> WORLD
    # =========================================================

    def grid_to_world(self, gx, gy):

        x = (
            self.min_x
            + (gx + 0.5)
            * self.resolution
        )

        y = (
            self.min_y
            + (gy + 0.5)
            * self.resolution
        )

        return x, y

    # =========================================================
    # GRID VALIDATION
    # =========================================================

    def is_valid_grid(self, gx, gy):

        return (
            0 <= gx < self.width
            and
            0 <= gy < self.height
        )

    # =========================================================
    # OCCUPANCY CHECK
    # =========================================================

    def is_occupied_grid(self, gx, gy):

        if not self.is_valid_grid(gx, gy):
            return True

        return self.grid[gx][gy]

    # =========================================================
    # ADD RECTANGULAR OBSTACLE
    # =========================================================

    def add_rectangle(
        self,
        name,
        center_x,
        center_y,
        size_x,
        size_y,
        inflation=None
    ):

        if inflation is None:
            inflation = self.robot_inflation

        half_x = (
            size_x / 2.0
            + inflation
        )

        half_y = (
            size_y / 2.0
            + inflation
        )

        min_x = center_x - half_x
        max_x = center_x + half_x

        min_y = center_y - half_y
        max_y = center_y + half_y

        self.obstacles.append(
            {
                'name': name,
                'center': (
                    center_x,
                    center_y
                ),
                'size': (
                    size_x,
                    size_y
                ),
                'inflation': inflation
            }
        )

        start_gx, start_gy = self.world_to_grid(
            min_x,
            min_y
        )

        end_gx, end_gy = self.world_to_grid(
            max_x,
            max_y
        )

        start_gx = max(
            0,
            start_gx
        )

        start_gy = max(
            0,
            start_gy
        )

        end_gx = min(
            self.width - 1,
            end_gx
        )

        end_gy = min(
            self.height - 1,
            end_gy
        )

        for gx in range(
            start_gx,
            end_gx + 1
        ):

            for gy in range(
                start_gy,
                end_gy + 1
            ):

                self.grid[gx][gy] = True

    # =========================================================
    # NORMAL SHELF
    # =========================================================

    def add_normal_shelf(
        self,
        name,
        x,
        y
    ):

        # Gazebo shelf model:
        #
        # link pose = (-0.5, 0)
        # collision = 3.6 x 0.6 m
        #
        # World collision center therefore:
        # x = model_x - 0.5

        collision_x = x - 0.5
        collision_y = y

        self.add_rectangle(
            name=name,
            center_x=collision_x,
            center_y=collision_y,
            size_x=3.6,
            size_y=0.6
        )

    # =========================================================
    # LARGE SHELF
    # =========================================================

    def add_large_shelf(
        self,
        name,
        x,
        y
    ):

        # Gazebo shelf_big model:
        #
        # link pose = (-0.5, 0)
        # collision = 2.1 x 18.0 m

        collision_x = x - 0.5
        collision_y = y

        self.add_rectangle(
            name=name,
            center_x=collision_x,
            center_y=collision_y,
            size_x=2.1,
            size_y=18.0
        )

    # =========================================================
    # BUILD STATIC WAREHOUSE MAP
    # =========================================================

    def build_static_map(self):

        # =====================================================
        # NORMAL SHELVES
        # =====================================================

        normal_shelves = [

            (
                'shelf',
                -4.41528,
                -0.690987
            ),

            (
                'shelf_0',
                -4.41528,
                2.30697
            ),

            (
                'shelf_1',
                -4.41528,
                5.30708
            ),

            (
                'shelf_2',
                -4.41528,
                8.34352
            ),

            (
                'shelf_3',
                5.60144,
                8.34352
            ),

            (
                'shelf_4',
                5.60144,
                5.30708
            ),

            (
                'shelf_5',
                5.60144,
                -0.690987
            ),

            (
                'shelf_6',
                5.60144,
                2.30697
            ),

            (
                'shelf_7',
                13.3818,
                -21.2416
            ),

            (
                'shelf_8',
                13.3818,
                -19.0028
            ),

            (
                'shelf_9',
                13.3818,
                -16.4478
            ),

            (
                'shelf_10',
                13.3818,
                -14.1028
            ),
        ]

        for name, x, y in normal_shelves:

            self.add_normal_shelf(
                name,
                x,
                y
            )

        # =====================================================
        # LARGE SHELVES
        # =====================================================

        large_shelves = [

            (
                'shelf_big_0',
                -9.34177,
                -13.5598
            ),

            (
                'shelf_big_1',
                13.9821,
                15.319
            ),

            (
                'shelf_big_2',
                6.19777,
                -12.9647
            ),

            (
                'shelf_big_3',
                0.594376,
                -12.9647
            ),

            (
                'shelf_big_4',
                -5.36284,
                -12.9647
            ),
        ]

        for name, x, y in large_shelves:

            self.add_large_shelf(
                name,
                x,
                y
            )

    # =========================================================
    # DEBUG SUMMARY
    # =========================================================

    def print_summary(self):

        occupied = 0

        for gx in range(self.width):

            for gy in range(self.height):

                if self.grid[gx][gy]:
                    occupied += 1

        total = (
            self.width
            * self.height
        )

        print(
            '========================================'
        )

        print(
            'WAREHOUSE OCCUPANCY MAP'
        )

        print(
            '========================================'
        )

        print(
            f'Resolution: '
            f'{self.resolution:.2f} m'
        )

        print(
            f'World X: '
            f'{self.min_x:.1f} '
            f'to {self.max_x:.1f}'
        )

        print(
            f'World Y: '
            f'{self.min_y:.1f} '
            f'to {self.max_y:.1f}'
        )

        print(
            f'Grid size: '
            f'{self.width} x '
            f'{self.height}'
        )

        print(
            f'Total cells: {total}'
        )

        print(
            f'Occupied cells: {occupied}'
        )

        print(
            f'Robot inflation: '
            f'{self.robot_inflation:.2f} m'
        )

        print(
            f'Static obstacles: '
            f'{len(self.obstacles)}'
        )

        print(
            '========================================'
        )

        for obstacle in self.obstacles:

            print(
                f"{obstacle['name']:15s} "
                f"center="
                f"("
                f"{obstacle['center'][0]:.2f}, "
                f"{obstacle['center'][1]:.2f}"
                f") "
                f"size="
                f"("
                f"{obstacle['size'][0]:.2f}, "
                f"{obstacle['size'][1]:.2f}"
                f")"
            )


if __name__ == '__main__':

    warehouse = WarehouseMap()

    warehouse.print_summary()