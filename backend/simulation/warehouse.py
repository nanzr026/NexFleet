"""Warehouse digital twin — 2D discrete-grid environment."""
from __future__ import annotations
import json
import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Set, Tuple


class CellType(str, Enum):
    EMPTY = "EMPTY"
    WALL = "WALL"
    RACK = "RACK"
    PICKUP = "PICKUP"
    DELIVERY = "DELIVERY"
    CHARGING = "CHARGING"
    OBSTACLE = "OBSTACLE"


@dataclass
class Station:
    station_id: str
    cell_type: CellType
    row: int
    col: int

    def pos(self) -> Tuple[int, int]:
        return (self.row, self.col)


class Warehouse:
    """30x30 warehouse grid with racks, aisles, stations."""

    DEFAULT_LAYOUT = """\
WWWWWWWWWWWWWWWWWWWWWWWWWWWWWW
W............................W
W.RRRR..RRRR..RRRR..RRRR....W
W............................W
W.RRRR..RRRR..RRRR..RRRR....W
W............................W
W.RRRR..RRRR..RRRR..RRRR....W
W............................W
W.RRRR..RRRR..RRRR..RRRR....W
W............................W
W.RRRR..RRRR..RRRR..RRRR....W
W............................W
W.RRRR..RRRR..RRRR..RRRR....W
W............................W
W.RRRR..RRRR..RRRR..RRRR....W
W............................W
W.RRRR..RRRR..RRRR..RRRR....W
W............................W
W.RRRR..RRRR..RRRR..RRRR....W
W............................W
W.RRRR..RRRR..RRRR..RRRR....W
W............................W
W.RRRR..RRRR..RRRR..RRRR....W
W............................W
WPPPP...PPPP....................W
W............................W
WDDDD...DDDD....................W
WCC..............................W
W............................W
WWWWWWWWWWWWWWWWWWWWWWWWWWWWWW"""

    def __init__(self, rows: int = 30, cols: int = 30):
        self.rows = rows
        self.cols = cols
        self.grid: List[List[CellType]] = []
        self.stations: Dict[str, Station] = {}
        self.dynamic_obstacles: Set[Tuple[int, int]] = set()
        self._build_grid()

    def _build_grid(self):
        """Build the default warehouse layout."""
        self.grid = [[CellType.EMPTY] * self.cols for _ in range(self.rows)]

        # Outer walls
        for r in range(self.rows):
            self.grid[r][0] = CellType.WALL
            self.grid[r][self.cols - 1] = CellType.WALL
        for c in range(self.cols):
            self.grid[0][c] = CellType.WALL
            self.grid[self.rows - 1][c] = CellType.WALL

        # Storage racks in a 4-column block pattern
        rack_rows = [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22]
        rack_col_groups = [(2, 6), (8, 12), (14, 18), (20, 24)]
        for r in rack_rows:
            for (cs, ce) in rack_col_groups:
                for c in range(cs, min(ce, self.cols - 1)):
                    self.grid[r][c] = CellType.RACK

        # Pickup stations P1-P4 (row 24)
        pickup_cols = [2, 5, 9, 13]
        for i, c in enumerate(pickup_cols):
            sid = f"P{i+1}"
            self.grid[24][c] = CellType.PICKUP
            self.stations[sid] = Station(sid, CellType.PICKUP, 24, c)

        # Delivery stations D1-D4 (row 26)
        delivery_cols = [2, 5, 9, 13]
        for i, c in enumerate(delivery_cols):
            sid = f"D{i+1}"
            self.grid[26][c] = CellType.DELIVERY
            self.stations[sid] = Station(sid, CellType.DELIVERY, 26, c)

        # Charging stations C1-C2 (row 28)
        charging_cols = [2, 5]
        for i, c in enumerate(charging_cols):
            sid = f"C{i+1}"
            self.grid[28][c] = CellType.CHARGING
            self.stations[sid] = Station(sid, CellType.CHARGING, 28, c)

    def is_passable(self, row: int, col: int) -> bool:
        """Return True if a robot can occupy this cell."""
        if not (0 <= row < self.rows and 0 <= col < self.cols):
            return False
        ct = self.grid[row][col]
        if ct in (CellType.WALL, CellType.RACK):
            return False
        if (row, col) in self.dynamic_obstacles:
            return False
        return True

    def add_obstacle(self, row: int, col: int) -> bool:
        """Add a dynamic obstacle. Returns True if successful."""
        if not (0 <= row < self.rows and 0 <= col < self.cols):
            return False
        if self.grid[row][col] in (CellType.WALL, CellType.RACK):
            return False
        self.dynamic_obstacles.add((row, col))
        return True

    def remove_obstacle(self, row: int, col: int) -> bool:
        if (row, col) in self.dynamic_obstacles:
            self.dynamic_obstacles.discard((row, col))
            return True
        return False

    def neighbors(self, row: int, col: int) -> List[Tuple[int, int]]:
        """Return passable 4-connected neighbors."""
        result = []
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = row + dr, col + dc
            if self.is_passable(nr, nc):
                result.append((nr, nc))
        return result

    def cell_type_at(self, row: int, col: int) -> CellType:
        if (row, col) in self.dynamic_obstacles:
            return CellType.OBSTACLE
        return self.grid[row][col]

    def get_station(self, station_id: str) -> Optional[Station]:
        return self.stations.get(station_id)

    def to_dict(self) -> dict:
        return {
            "rows": self.rows,
            "cols": self.cols,
            "stations": {k: {"id": v.station_id, "type": v.cell_type, "row": v.row, "col": v.col}
                         for k, v in self.stations.items()},
            "dynamic_obstacles": list(self.dynamic_obstacles),
            "grid": [[ct.value for ct in row] for row in self.grid],
        }

    def get_pickup_stations(self) -> List[Station]:
        return [s for s in self.stations.values() if s.cell_type == CellType.PICKUP]

    def get_delivery_stations(self) -> List[Station]:
        return [s for s in self.stations.values() if s.cell_type == CellType.DELIVERY]
