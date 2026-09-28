import copy
import math
import pytest
import torch
from bel_amr_ros.fleet_graph import IDS, SCHEMA, snapshot, advisory_bid, digest
from bel_amr_ros.graph_learning import initial_model, weights, load, train, predict, fedavg, ARCH
from bel_amr_ros.graph_experience import label_graphs

def graph():
    states={r:dict(x=i*4.,y=0.,vx=0.,vy=0.) for i,r in enumerate(IDS)}
    return snapshot(states,10.,{IDS[0]:[4.,0.]},"test_measurements")

def test_physical_edges_and_missing_measurements():
    g=graph()
    assert len(g["edges"])==4
    assert all(e["distance"]==4 for e in g["edges"])
    assert not any(e["source"]==IDS[0] and e["target"]==IDS[2] for e in g["edges"])
    with pytest.raises(KeyError):snapshot({},0,{},"missing")
    bad={r:dict(x=0,y=0,vx=float("nan"),vy=0) for r in IDS}
    with pytest.raises(ValueError):snapshot(bad,0,{},"bad")

def test_message_path_and_trainable_parameters():
    model=initial_model();g=graph()
    assert sum(p.numel() for p in model.parameters())<1000
    assert predict(model,g,IDS[0])!=predict(model,g,IDS[0],True)
    original=digest(weights(model))
    samples=[dict(graph=g,robot_id=IDS[0],target=[.1,.9])]
    metrics=train(model,samples,epochs=30)
    assert metrics["after"]<metrics["before"]*.2
    assert digest(weights(model))!=original

def test_disconnected_node_cannot_message_ego():
    model=initial_model();g=graph();edited=copy.deepcopy(g)
    edited["nodes"][IDS[2]]["features"]=[100.]*7
    assert predict(model,g,IDS[0])==predict(model,edited,IDS[0])

def test_weighted_fedavg_and_rejections():
    base=weights(initial_model());parent=digest(base)
    updates=[dict(robot_id=r,run="run",version=1,parent=parent,architecture=ARCH,
                  samples=i+1,weights={k:[float(i)]*len(v) for k,v in base.items()})
             for i,r in enumerate(IDS)]
    result=fedavg(updates,parent,1,"run")
    assert all(abs(v-8/6)<1e-6 for values in result.values() for v in values)
    for key,value in (("parent","stale"),("architecture","wrong"),("samples",0)):
        bad=copy.deepcopy(updates);bad[0][key]=value
        with pytest.raises(ValueError):fedavg(bad,parent,1,"run")
    with pytest.raises(ValueError):fedavg(updates[:2],parent,1,"run")
    bad=copy.deepcopy(updates);next(iter(bad[0]["weights"].values()))[0]=math.nan
    with pytest.raises(ValueError):fedavg(bad,parent,1,"run")

def test_advice_is_bounded_and_fails_to_exact_baseline():
    good=dict(task_id="t",time=100,version=3,schema=SCHEMA,suitability=1.,risk=0.)
    assert advisory_bid(.7,good,"t",101)==pytest.approx(.74)
    for field,value in (("task_id","other"),("time",90),("version",0),
                        ("suitability",math.nan),("risk",2),("schema","bad")):
        bad=dict(good);bad[field]=value
        assert advisory_bid(.7,bad,"t",101)==.7
    assert advisory_bid(.7,{},"t",101)==.7

def test_labels_use_observed_future_and_duration():
    g=graph()
    graphs=[]
    for t in (0.,1.,3.):
        item=copy.deepcopy(g);item["timestamp"]=t
        item["nodes"][IDS[1]]["x"]=1. if t==0 else 4.
        graphs.append(item)
    rows=label_graphs(graphs,IDS[0],3.)
    assert rows[0]["target"]==pytest.approx([3/180,1/3])
    assert rows[1]["target"]==pytest.approx([2/180,0])

def test_parameters_round_trip_reject_shape():
    model=initial_model();state=weights(model);other=initial_model()
    load(other,state)
    assert digest(weights(other))==digest(state)
    state["head.bias"]=[0.]
    with pytest.raises(ValueError):load(other,state)

def test_invalid_goal_rejected_before_inference():
    states={r:dict(x=0,y=0,vx=0,vy=0) for r in IDS}
    with pytest.raises(ValueError):
        snapshot(states,0,{IDS[0]:[math.nan,0]},"test")
