"""Webots ENU occupancy map. Geometry comes from the Webots layout, never Gazebo."""
import json,math
from pathlib import Path
from .warehouse_map import WarehouseMap
LAYOUT_PATH=Path(__file__).resolve().parents[4]/'Webots/config/warehouse_layout.json'
def load_layout(path=None):
    return json.loads(Path(path or LAYOUT_PATH).read_text())
class WebotsWarehouseMap(WarehouseMap):
    def __init__(self,robot_inflation=.60,layout_path=None):
        self.layout=load_layout(layout_path)
        self.resolution=self.layout['resolution']
        self.min_x,self.max_x=self.layout['bounds']['x']
        self.min_y,self.max_y=self.layout['bounds']['y']
        self.width=math.ceil((self.max_x-self.min_x)/self.resolution)
        self.height=math.ceil((self.max_y-self.min_y)/self.resolution)
        self.robot_inflation=robot_inflation
        self.grid=[[False]*self.height for _ in range(self.width)]
        self.obstacles=[];self.dynamic_cells={};self.peer_poses={}
        self.pallets={p['name']:(p['x'],p['y'],0.0) for p in self.layout['pallets']}
        self.build_static_map()
    def build_static_map(self):
        for o in self.layout['racks']+self.layout['docks']+self.layout['obstacles']:
            self.add_rectangle(o['name'],o['x'],o['y'],*o['size'])
        for name,x,y,sx,sy in [
            ('north',0,self.max_y,40,.2),('south',0,self.min_y,40,.2),
            ('west',self.min_x,0,.2,32),('east',self.max_x,0,.2,32)]:
            self.add_rectangle(name,x,y,sx,sy)
