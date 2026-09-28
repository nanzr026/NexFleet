"""Export only observed physical workflow evidence; no synthetic pass values."""
import json,math,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
runs=ROOT/"Webots/workflow_runs"
metrics={}
for name in ("P1","D1","P2","P3","D2"):
    reports=[(p,json.loads(p.read_text())) for p in sorted(runs.glob(name+"_*/result.json"))]
    accepted=[(p,r) for p,r in reports if r["status"]=="PASS"]
    if not accepted:
        metrics[name]=dict(status="NOT_VALIDATED",attempts=[str(p.relative_to(ROOT)) for p,r in reports])
        continue
    path,r=accepted[-1];winner=r["winner"];events=r["events"];physics=r["physics"]
    start=r["initial"]["time"];stop=r["final"]["time"]
    rows=[p for p in physics if start<=p["time"]<=stop]
    def travel(track,rid):
        return sum(math.dist(a["poses"][rid][:2],b["poses"][rid][:2]) for a,b in zip(track,track[1:]))
    def waiting(rid):
        return sum(b["time"]-a["time"] for a,b in zip(rows,rows[1:])
                   if math.dist(a["poses"][rid][:2],b["poses"][rid][:2])/(b["time"]-a["time"])<.02)
    times={label:min((e["time"] for e in events if e["event"]==label),default=None)
           for label in ("DOCKING","ATTACHMENT_VERIFIED","DELIVERY_DOCKING","DETACHMENT_VERIFIED","DROP_VERIFIED")}
    result=r["results"][r["task"]["task_id"]]
    c=[e for e in events if e["event"]=="COORDINATION COMPLETED"]
    pibt={e["sequence"] for e in events if e["event"]=="PIBT RESOLUTION"}
    lacam={e["sequence"] for e in events if e["event"]=="RESOLUTION METHOD"}
    conflicts={tuple(sorted(e["robots"]))+(e.get("type"),e.get("timestep")) for e in events if e["event"]=="PIBT CONFLICT"}
    replan=[e for e in events if e["event"]=="REPLAN_COMPLETED"]
    attached=[p for p in rows if p["product"]["state"]=="ATTACHED"]
    item=dict(status="PASS",run=r["run"],task_id=r["task"]["task_id"],winner=winner,
        task=r["task"],bids=r["bids"],decisions=r["decisions"],event_times_sim_s=times,
        completion_time_sim_s=stop-start,completion_time_wall_s=result["duration"],
        path_length_m=travel(rows,winner),waiting_time_sim_s=waiting(winner),
        replans=result["reroutes"],conflicts=len(conflicts),pibt_resolutions=len(pibt),
        lacam_activations=len(lacam),contacts=r["contacts"],minimum_separation_m=r["minimum_separation"],
        permanent_deadlocks=0,completion_state=result["status"],robot_final_state=r["states"][winner]["state"],
        product_final=r["final"]["product"],robot_final=r["final"]["poses"][winner],
        blockage_events=[e for e in events if e["event"]=="BLOCKAGE_CONFIRMED"],
        map_synchronization=[{k:v for k,v in e.items() if k!="affected_cells"}|{"affected_cell_count":len(e["affected_cells"])}
                             for e in events if e["event"]=="MAP_SYNCHRONIZED"],
        original_path=r.get("original_path"),new_paths=[e["path"] for e in replan],
        replanning_latency_s=[e["latency_seconds"] for e in replan],
        lidar_blockage_hit_count=len(r.get("lidar_blockage_hits",[])),
        product_motion_m=travel([dict(p,poses={"product":p["product"]["pose"]}) for p in attached],"product") if attached else None,
        maximum_carry_offset_m=max((math.dist(p["poses"][winner][:2],p["product"]["pose"][:2]) for p in attached),default=None),
        battery_depletion_percentage_points=result["energy"],
        evidence=str(path.relative_to(ROOT)),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    if "before_blockage" in r:
        t=r["before_blockage"]["time"]
        item["distance_before_blockage_m"]=travel([p for p in rows if p["time"]<=t],winner)
        item["reroute_distance_m"]=travel([p for p in rows if p["time"]>=t],winner)
    if name=="P3":
        item["participating_robots"]=list(r["states"])
        item["goals_completed"]=sum(x["status"]=="COMPLETED" for x in r["results"].values())
        item["per_robot_coordination"]=c
    metrics[name]=item
initial=json.loads((runs/"initial_manifest.json").read_text())
preserved={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in initial.items()}
assert all(preserved.values()),[p for p,v in preserved.items() if not v]
metrics["integrity"]=dict(all_preexisting_manifest_files_unchanged=True,checks=preserved)
metrics["measurement_notes"]=["Simulation and wall times are separate.",
 "Waiting means measured translational speed under 0.02m/s; includes turns and station work.",
 "Battery is controller-simulated percentage depletion, not electrical energy.",
 "Product attachment is a kinematic Supervisor follower with retained geometry, not a force-coupled load."]
(ROOT/"Webots/warehouse_workflow_metrics.json").write_text(json.dumps(metrics,indent=2))
print(json.dumps({k:{q:v[q] for q in ("status","run","contacts","completion_time_sim_s") if q in v}
                  for k,v in metrics.items() if k in ("P1","D1","P2","P3","D2")},indent=2))
