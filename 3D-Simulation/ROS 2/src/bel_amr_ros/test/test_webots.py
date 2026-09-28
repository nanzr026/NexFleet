"""World/map consistency and reuse checks for the Webots migration."""
import math,re
from pathlib import Path
from bel_amr_ros.webots_map import WebotsWarehouseMap,LAYOUT_PATH,load_layout
from bel_amr_ros.astar_planner import AStarPlanner
def test_world_geometry_and_map_share_coordinates():
    layout=load_layout()
    world=(LAYOUT_PATH.parents[1]/'worlds/bel_warehouse.wbt').read_text()
    m=WebotsWarehouseMap()
    for o in layout['racks']+layout['docks']:
        pattern=rf'translation {o["x"]} {o["y"]} 0 name "{o["name"]}"'
        assert pattern in world
        assert m.is_occupied_grid(*m.world_to_grid(o['x'],o['y']))
    assert (m.min_x,m.max_x,m.min_y,m.max_y)==(-20,20,-16,16)
def test_all_spawn_locations_are_free_and_route_to_main_aisle():
    m=WebotsWarehouseMap();planner=AStarPlanner(m)
    for r in load_layout()['robots']:
        start=(r['x'],r['y'])
        assert not m.is_occupied_grid(*m.world_to_grid(*start))
        path=planner.plan(start,(0,1.5))
        assert path and all(planner.has_line_of_sight(a,b) for a,b in zip(path,path[1:]))
def test_active_world_assets_have_no_network_references():
    root=LAYOUT_PATH.parents[1]
    files=[root/'worlds/bel_warehouse.wbt',root/'worlds/bel_warehouse_ros.wbt',*list((root/'protos').glob('*.proto'))]
    for file in files:
        assert 'https://' not in file.read_text()
        for uri in re.findall(r'EXTERNPROTO "([^"]+)"',file.read_text()):
            assert (file.parent/uri).exists()
def test_dynamic_obstacle_changes_the_webots_route():
    m=WebotsWarehouseMap();p=AStarPlanner(m)
    start,goal=(-5,1.5),(5,1.5)
    direct=p.plan(start,goal)
    m.add_dynamic_obstacle(0,1.5,size=1.0,ttl=60)
    detour=p.plan(start,goal)
    assert detour!=direct
    assert all(p.has_line_of_sight(a,b) for a,b in zip(detour,detour[1:]))
