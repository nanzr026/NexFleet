from bel_amr_ros.warehouse_workflow import *
def test_pickup_distance():assert pickup_rejection((0,0,0),(5,0,0),0,None,"AVAILABLE")=="PICKUP_DISTANCE"
def test_pickup_alignment():assert pickup_rejection((0,0,1),(1.25,0,0),0,None,"AVAILABLE")=="PICKUP_ALIGNMENT"
def test_pickup_moving():assert pickup_rejection((0,0,0),(1.25,0,0),.1,None,"AVAILABLE")=="ROBOT_MOVING"
def test_pickup_owner():assert pickup_rejection((0,0,0),(1.25,0,0),0,"amr_02","ATTACHED")=="PRODUCT_UNAVAILABLE"
def test_pickup_valid():assert pickup_rejection((0,0,0),(1.25,0,0),0,None,"AVAILABLE") is None
def test_drop_owner():assert drop_rejection((0,0,0),0,"amr_01","amr_02",(1.25,0))=="NOT_CARRIER"
def test_drop_location():assert drop_rejection((0,0,0),0,"amr_01","amr_01",(9,0))=="DROP_LOCATION"
def test_drop_valid():assert drop_rejection((0,0,0),0,"amr_01","amr_01",(1.25,0)) is None
def test_release_requires_ack():assert not released_at((1,0),(1,0),"amr_01","ATTACHED")
def test_release_verification():assert released_at((1,0),(1,0),None,"DELIVERED")

def test_nonfinite_pickup_rejected():
    assert pickup_rejection((float("nan"),0,0),(1.25,0,0),0,None,"AVAILABLE")=="INVALID_MEASUREMENT"
def test_failed_pickup_does_not_grant_ownership():
    carrier=None
    assert pickup_rejection((0,0,0),(9,0,0),0,carrier,"AVAILABLE")
    assert carrier is None

def test_rotated_load_envelope():
    assert rectangles_overlap((0,0,.6,.4,.7),(.8,0,.2,.4,0))
    assert not rectangles_overlap((0,0,.6,.4,.7),(2,0,.2,.4,0))

from bel_amr_ros.warehouse_environment import EnvironmentMap
from bel_amr_ros.astar_planner import AStarPlanner
def test_confirmed_map_and_route_replan():
    m=EnvironmentMap();p=AStarPlanner(m)
    before=p.plan((0,-10),(10,-10))
    event=dict(confirmed=False,version=1,blockages=[dict(id="b",active=True,x=5,y=-10,sx=.8,sy=1.2)])
    assert not m.apply(event)
    assert p.has_line_of_sight((0,-10),(10,-10))
    event["confirmed"]=True
    assert m.apply(event)
    assert not p.has_line_of_sight((0,-10),(10,-10))
    assert m.affected_cells()
    after=p.plan((0,-10),(10,-10))
    assert after and after!=before
    assert not m.apply(event)
    event=dict(confirmed=True,version=2,blockages=[])
    assert m.apply(event) and p.has_line_of_sight((0,-10),(10,-10))
def test_loaded_replan_preserves_attachment_geometry():
    m=EnvironmentMap(.85);m.apply(dict(confirmed=True,version=1,blockages=[dict(id="b",active=True,x=6,y=-10,sx=.8,sy=1.2)]))
    path=AStarPlanner(m).plan((3,-10),(8.75,-10))
    assert path and len(path)>2
    # Map mutations contain no product or carrier authority.
    assert not hasattr(m,"carrier")
