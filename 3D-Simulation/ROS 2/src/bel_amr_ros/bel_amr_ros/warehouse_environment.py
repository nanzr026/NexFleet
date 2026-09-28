"""Confirmed environment occupancy layered over the accepted Webots map."""
import math
from .webots_map import WebotsWarehouseMap
class EnvironmentMap(WebotsWarehouseMap):
    def __init__(self,robot_inflation=.60):
        super().__init__(robot_inflation)
        self.blockages={};self.version=0
    def apply(self,event):
        if not event.get("confirmed") or event["version"]<=self.version:return False
        blocks={}
        for b in event["blockages"]:
            if b["active"]:blocks[b["id"]]=b
        self.blockages=blocks;self.version=event["version"];return True
    def is_occupied_grid(self,gx,gy):
        if super().is_occupied_grid(gx,gy):return True
        x,y=self.grid_to_world(gx,gy)
        # Loaded controller retains its existing 1.85m all-direction LiDAR stopping margin.
        margin=2.0 if self.robot_inflation>.8 else self.robot_inflation
        return any(abs(x-b["x"])<=b["sx"]/2+margin and abs(y-b["y"])<=b["sy"]/2+margin for b in self.blockages.values())
    def affected_cells(self):
        margin=2.0 if self.robot_inflation>.8 else self.robot_inflation
        return [[gx,gy] for gx in range(self.width) for gy in range(self.height)
                if any(abs(self.grid_to_world(gx,gy)[0]-b["x"])<=b["sx"]/2+margin and
                       abs(self.grid_to_world(gx,gy)[1]-b["y"])<=b["sy"]/2+margin
                       for b in self.blockages.values())]
