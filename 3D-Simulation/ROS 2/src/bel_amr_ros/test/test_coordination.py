import math
from bel_amr_ros.coordination_search import ConfigurationSearch, swept_distance

def corridor():
    return ConfigurationSearch([(5,-6),(5,-4),(5,-2),(5,0),(3,-6),(5,-8)],
                               [(0,1),(1,2),(2,3),(0,4),(0,5)], [3,5], [145,132])

def test_continuous_collision_between_samples():
    assert swept_distance((-1,0),(1,0),(0,-1),(0,1)) == 0
    assert swept_distance((0,0),(2,0),(2,0),(0,0)) == 0

def test_pibt_inherits_and_backtracks():
    s=corridor()
    q=s.pibt((0,3))
    assert q and s.compatible((0,3),q)
    q2=s.pibt(q)
    s.pibt(q2)
    assert s.inheritances > 0 and s.backtracks > 0
    assert q2 is None or s.compatible(q,q2)

def test_lacam_escapes_greedy_corridor_deadlock():
    s=corridor()
    path,expanded=s.lacam((1,2))
    assert path and path[-1]==s.goals and expanded>1
    assert all(s.compatible(a,b) for a,b in zip(path,path[1:]))
    assert any(4 in q for q in path), path

def test_unsolvable_swap_returns_no_path():
    s=ConfigurationSearch([(0,0),(2,0)],[(0,1)],[1,0],[2,1])
    path,_=s.lacam((0,1))
    assert path is None

def test_lacam_constraints_are_respected():
    s=corridor()
    result=s.pibt((0,3),(4,))
    assert result and result[0]==4

def test_search_budget_fails_closed():
    path,_=corridor().lacam((0,3),max_expansions=0)
    assert path is None


def test_dds_barrier_rejects_mismatched_decision_and_stale_peer():
    from types import SimpleNamespace
    from bel_amr_ros.webots_coordination import CoordinationSession
    class Pub:
        def publish(self, msg):pass
    class Log:
        def info(self, msg):pass
    spec=dict(run='test',members=['amr_01','amr_02'],
              nodes=[(0,0),(2,0),(4,0),(2,2)],edges=[(0,1),(1,2),(1,3)])
    def node(rid,x,y):
        return SimpleNamespace(robot_id=rid,x=x,y=y,battery_level=90.,
             pose_sim_time=1.,astar=SimpleNamespace(has_line_of_sight=lambda a,b:True),
             coordination_events=Pub(),get_logger=lambda:Log())
    a=CoordinationSession(node('amr_01',0,0),
        dict(coordination=spec,pickup_x=4,pickup_y=0),[(0,0),(4,0)])
    b=CoordinationSession(node('amr_02',2,2),
        dict(coordination=spec,pickup_x=0,pickup_y=0),[(2,2),(0,0)])
    a.receive('amr_02',b.state());b.receive('amr_01',a.state())
    a.update();b.update()
    bad=b.state();bad['signature']='wrong'
    a.receive('amr_02',bad);a.update()
    assert not a.allow
    a.receive('amr_02',b.state())
    state,when=a.peers['amr_02']
    a.peers['amr_02']=(state,when-10)
    a.update()
    assert not a.allow

def test_graph_rejects_blocked_edges():
    import pytest
    from types import SimpleNamespace
    from bel_amr_ros.webots_coordination import CoordinationSession
    n=SimpleNamespace(robot_id='amr_01',astar=SimpleNamespace(has_line_of_sight=lambda a,b:False))
    task=dict(coordination=dict(run='bad',members=['amr_01','amr_02'],
                               nodes=[(0,0),(2,0)],edges=[(0,1)]))
    with pytest.raises(ValueError,match='occupancy'):
        CoordinationSession(n,task,[(0,0),(2,0)])


def test_independent_sessions_agree_and_finish_corridor_with_fallback():
    from types import SimpleNamespace
    from bel_amr_ros.webots_coordination import CoordinationSession
    class Pub:
        def publish(self,msg):pass
    class Log:
        def info(self,msg):pass
    search=corridor()
    spec=dict(run='distributed',members=['amr_01','amr_02'],nodes=search.nodes,
              edges=[(0,1),(1,2),(2,3),(0,4),(0,5)])
    sessions=[]
    for i,(start,goal) in enumerate(zip([0,3],[3,5])):
        rid=f'amr_{i+1:02}'
        x,y=search.nodes[start]
        n=SimpleNamespace(robot_id=rid,x=x,y=y,battery_level=90.-i*25,
            pose_sim_time=1.,astar=SimpleNamespace(has_line_of_sight=lambda a,b:True),
            coordination_events=Pub(),get_logger=lambda:Log())
        gx,gy=search.nodes[goal]
        sessions.append(CoordinationSession(n,dict(coordination=spec,pickup_x=gx,pickup_y=gy),
                        [search.nodes[start],search.nodes[goal]]))
    for tick in range(500):
        snapshots=[s.state() for s in sessions]
        for i,s in enumerate(sessions):
            s.n.pose_sim_time+=.2
            s.receive(sessions[1-i].n.robot_id,snapshots[1-i])
            s.update()
        for s in sessions:
            if s.allow:
                s.n.x,s.n.y=s.nodes[s.target[s.index]]
        if all(s.done for s in sessions):break
    assert all(s.done for s in sessions), [s.state() for s in sessions]
    assert all(s.metrics['lacam_activations']==1 for s in sessions)
    assert all(s.metrics['deadlocks']==1 for s in sessions)


def test_pibt_prefers_safe_detour_over_waiting_at_crossing():
    nodes=[(x,y) for y in (-12,-10,-8) for x in (0,2,4)]
    edges=[(i,j) for i,a in enumerate(nodes) for j,b in enumerate(nodes[:i]) if math.dist(a,b)==2]
    s=ConfigurationSearch(nodes,edges,[5,7],[145,132])
    result=s.pibt((3,1))
    assert result[0]==4 and result[1]!=1
    assert s.compatible((3,1),result)


def test_three_sessions_with_unequal_execution_speeds():
    from types import SimpleNamespace
    from bel_amr_ros.webots_coordination import CoordinationSession
    class Pub:
        def publish(self,msg):pass
    class Log:
        def info(self,msg):pass
    nodes=[(x,y) for y in (-12,-10,-8) for x in (0,2,4)]
    edges=[(i,j) for i,a in enumerate(nodes) for j,b in enumerate(nodes[:i]) if math.dist(a,b)==2]
    ids=['amr_01','amr_02','amr_03']
    spec=dict(run='three',members=ids,nodes=nodes,edges=edges)
    sessions=[]
    for i,(start,goal) in enumerate(zip([3,1,5],[5,7,3])):
        x,y=nodes[start]
        n=SimpleNamespace(robot_id=ids[i],x=x,y=y,battery_level=90.-i*25,
            pose_sim_time=1.,astar=SimpleNamespace(has_line_of_sight=lambda a,b:True),
            coordination_events=Pub(),get_logger=lambda:Log())
        gx,gy=nodes[goal]
        sessions.append(CoordinationSession(n,dict(coordination=spec,pickup_x=gx,pickup_y=gy),
                        [nodes[start],nodes[goal]]))
    for tick in range(1600):
        snapshots=[s.state() for s in sessions]
        for i,s in enumerate(sessions):
            s.n.pose_sim_time+=.2
            for j,other in enumerate(sessions):
                if i!=j:s.receive(other.n.robot_id,snapshots[j])
            s.update()
        for i,s in enumerate(sessions):
            if s.allow:
                target=s.nodes[s.target[s.index]]
                length=math.dist((s.n.x,s.n.y),target)
                ratio=min(1.,(.08-.015*i)/length)
                s.n.x+=(target[0]-s.n.x)*ratio
                s.n.y+=(target[1]-s.n.y)*ratio
        assert min(math.dist((a.n.x,a.n.y),(b.n.x,b.n.y))
                   for i,a in enumerate(sessions) for b in sessions[:i])>1.25
        if all(s.done for s in sessions):break
    assert all(s.done for s in sessions),[s.state() for s in sessions]
