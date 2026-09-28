"""Labels from measured trajectories, never synthetic bootstrap."""
import math
from .fleet_graph import IDS, snapshot

def label_graphs(graphs, rid, completed):
    rows = []
    eligible = [g for g in graphs if g["timestamp"] <= completed]
    for i, graph in enumerate(eligible):
        t = graph["timestamp"]
        horizon = min(t+10, completed)
        exposure, duration = 0., 0.
        # Piecewise constant measured occupancy; time weighting tolerates dropped messages.
        for j in range(i, len(eligible)-1):
            a, b = eligible[j], eligible[j+1]
            dt = max(0., min(horizon, b["timestamp"])-max(t, a["timestamp"]))
            if dt:
                own = a["nodes"][rid]
                nearest = min(math.hypot(own["x"]-v["x"], own["y"]-v["y"])
                              for r,v in a["nodes"].items() if r != rid)
                duration += dt
                exposure += dt*float(nearest < 2.2)
            if b["timestamp"] >= horizon:
                break
        if duration > 0:
            rows.append(dict(robot_id=rid, graph=graph,
                             target=[min(1., max(0., completed-t)/180), exposure/duration]))
    return rows

def seed_samples(report, goals, rid):
    if report["status"] != "PASS" or report["collisions"] != 0:
        raise ValueError("Seed episode not accepted")
    if rid not in goals:
        return []
    times = [e["simulation_time"] for e in report["events"]
             if e["event"] == "SESSION"]
    ends = [e["simulation_time"] for e in report["events"]
            if e["robot_id"] == rid and e["event"] == "COMPLETED"]
    if not ends:
        # Controller reports COORDINATION COMPLETED in the accepted log.
        ends = [e["simulation_time"] for e in report["events"]
                if e["robot_id"] == rid and "COMPLETED" in e["event"]]
    if not ends:
        raise ValueError("Missing observed completion")
    graphs=[]; previous=None; last=-1e9
    for record in report["physics"]:
        if previous is not None and record["time"]-last >= 1 and record["time"] >= min(times):
            dt=record["time"]-previous["time"]
            if dt <= 0:
                continue
            states={}
            for r in IDS:
                x,y=record["poses"][r][:2]
                px,py=previous["poses"][r][:2]
                states[r]=dict(x=x,y=y,vx=(x-px)/dt,vy=(y-py)/dt)
            graphs.append(snapshot(states,record["time"],goals,"accepted_supervisor"))
            last=record["time"]
        previous=record
    return label_graphs(graphs,rid,max(ends))
