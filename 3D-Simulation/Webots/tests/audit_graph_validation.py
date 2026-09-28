"""Numerical FedAvg audit, identical-sample evaluation and measured CPU costs."""
import argparse
import csv
import hashlib
import json
import math
import platform
import statistics
import time
from pathlib import Path
import numpy as np
import torch
from bel_amr_ros.fleet_graph import IDS, SCHEMA
from bel_amr_ros.graph_learning import initial_model,load,predict,fedavg
ROOT=Path(__file__).resolve().parents[2]
parser=argparse.ArgumentParser()
parser.add_argument("run");parser.add_argument("capture");parser.add_argument("failsafe")
args=parser.parse_args()
directory=ROOT/"Webots/edge_ai_runs"/args.run
capture_dir=ROOT/"Webots/edge_ai_runs"/args.capture
fallback_dir=ROOT/"Webots/edge_ai_runs"/args.failsafe
def read(path):return json.loads(path.read_text())
def lines(path):return [json.loads(s) for s in path.read_text().splitlines()]
physical=read(directory/"physical_result.json")
capture=read(capture_dir/"capture.json")
fallback=read(fallback_dir/"failsafe_result.json")
assert physical["status"]==capture["status"]==fallback["status"]=="PASS"
metrics=read(ROOT/"Webots/edge_ai_metrics.json")
comparison=read(directory/"baseline_comparison.json")
evaluations={r:next(e for e in lines(directory/r/"evaluation.jsonl") if e["version"]==3) for r in IDS}
rows=[]
for rid,evaluation in evaluations.items():
    local=comparison["per_robot"][rid]["predictions"]
    assert len(local)==len(evaluation["predictions"])
    for p,b in zip(evaluation["predictions"],local):
        assert p["target"]==b["target"]
        rows.append(dict(robot=rid,timestamp=p["timestamp"],actual_seconds=p["target"][0]*180,
            predicted_seconds=p["gnn"][0]*180,actual_exposure=p["target"][1],
            predicted_exposure=p["gnn"][1],target=p["target"],
            full_graph=p["gnn"],no_neighbors=p["no_neighbors"],
            local_only=b["local_only"],simple=b["current_proximity"]))
def errors(items):
    return {key:dict(
        remaining_time_mae_s=statistics.mean(abs(p[key][0]-p["target"][0])*180 for p in items),
        remaining_time_rmse_s=math.sqrt(statistics.mean(((p[key][0]-p["target"][0])*180)**2 for p in items)),
        exposure_mae=statistics.mean(abs(p[key][1]-p["target"][1]) for p in items),
        exposure_rmse=math.sqrt(statistics.mean((p[key][1]-p["target"][1])**2 for p in items)))
        for key in ("full_graph","no_neighbors","local_only","simple")}
evaluation=dict(samples=len(rows),fleet=errors(rows),
    per_robot={r:dict(samples=sum(p["robot"]==r for p in rows),
                     models=errors([p for p in rows if p["robot"]==r])) for r in IDS})
with (directory/"frozen_predictions.csv").open("w",newline="") as f:
    fields=["robot","timestamp","actual_seconds","predicted_seconds","actual_exposure","predicted_exposure"]
    writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
    writer.writerows({k:p[k] for k in fields} for p in rows)
expected_keys={"run","robot_id","version","architecture","parent","samples","weights"}
rounds=[]
for version in (1,2,3):
    messages=[u for u in capture["updates"] if u["message"]["version"]==version]
    assert len(messages)==3 and {u["message"]["robot_id"] for u in messages}==set(IDS)
    updates=[u["message"] for u in messages]
    assert all(set(u)==expected_keys for u in updates)
    assert all(isinstance(x,(float,int)) for u in updates for v in u["weights"].values() for x in v)
    count=sum(u["samples"] for u in updates)
    # Independent scalar float64 sum; deliberately does not call implementation fedavg().
    expected={k:[math.fsum(u["samples"]*u["weights"][k][i] for u in updates)/count
                 for i in range(len(updates[0]["weights"][k]))] for k in updates[0]["weights"]}
    original=read(directory/"amr_01"/f"model_{version}.json")["weights"]
    actual=read(capture_dir/"amr_01"/f"model_{version}.json")["weights"]
    replay_difference=max(abs(actual[k][i]-original[k][i]) for k in original for i in range(len(original[k])))
    difference=max(abs(expected[k][i]-actual[k][i]) for k in actual for i in range(len(actual[k])))
    assert replay_difference==0 and difference<1e-6
    timings=[]
    for _ in range(30):
        start=time.perf_counter()
        fedavg(updates,updates[0]["parent"],version,args.capture)
        timings.append((time.perf_counter()-start)*1000)
    rounds.append(dict(round=version,sample_counts={u["robot_id"]:u["samples"] for u in updates},
        max_absolute_parameter_difference=difference,
        replay_vs_original_physical_run_parameter_difference=replay_difference,
        update_payload_bytes={u["message"]["robot_id"]:u["utf8_bytes"] for u in messages},
        total_update_payload_bytes=sum(u["utf8_bytes"] for u in messages),
        aggregation_mean_ms=statistics.mean(timings),aggregation_p95_ms=float(np.percentile(timings,95))))
training={r:lines(capture_dir/r/"training.jsonl") for r in IDS}
model=initial_model();load(model,read(directory/"amr_01"/"model_3.json")["weights"])
examples=[]
for rid in IDS:
    experience=next(e for e in lines(directory/rid/"local_experience.jsonl") if e["task_id"]==args.run+"_"+rid)
    examples.extend((rid,s["graph"]) for s in experience["rows"])
for rid,g in examples[:20]:predict(model,g,rid)
latencies=[]
for i in range(300):
    rid,g=examples[i%len(examples)]
    start=time.perf_counter();predict(model,g,rid)
    latencies.append((time.perf_counter()-start)*1000)
cost=dict(parameters=sum(p.numel() for p in model.parameters()),
    float32_parameter_bytes=sum(p.numel()*4 for p in model.parameters()),
    json_checkpoint_bytes=(directory/"amr_01"/"model_3.json").stat().st_size,
    inference_mean_ms=statistics.mean(latencies),inference_p95_ms=float(np.percentile(latencies,95)),
    inference_calls=len(latencies),inference_scope="CPU graph tensor preparation + all-node forward + ego extraction; warmed",
    torch_version=torch.__version__,python_version=platform.python_version(),
    cpu_threads=torch.get_num_threads(),training_measurement="Three concurrent DDS peers in seed-only replay",
    local_training_seconds={r:{str(t["version"]):t["duration_seconds"] for t in ts} for r,ts in training.items()})
starts={r:min(e["simulation_time"] for e in physical["events"] if e["robot_id"]==r and e["event"]=="SESSION") for r in IDS}
completed={e["robot_id"]:e for e in physical["events"] if e["event"]=="COORDINATION COMPLETED"}
robots={}
for r in IDS:
    stop=completed[r]["simulation_time"]
    track=[p["poses"][r][:2] for p in physical["physics"] if starts[r]<=p["time"]<=stop]
    distance=sum(math.dist(a,b) for a,b in zip(track,track[1:]))
    duration=stop-starts[r]
    result=physical["results"][args.run+"_"+r]
    robots[r]=dict(completed=True,completion_time_sim_s=duration,
        completion_time_wall_s=result["duration"],path_length_m=distance,
        average_speed_m_per_sim_s=distance/duration,final_position=physical["coordinated_final"]["poses"][r][:2],
        battery_depletion_percentage_points=result["energy"],**completed[r]["metrics"])
fleet=dict(goals_completed=3,goals_required=3,contacts=physical["collisions"],permanent_deadlocks=0,
    makespan_sim_s=max(e["simulation_time"] for e in completed.values())-min(starts.values()),
    minimum_physical_separation_m=physical["minimum_separation"],
    conflicts=len({tuple(sorted(e["robots"]))+(e["type"],e["timestep"]) for e in physical["events"] if e["event"]=="PIBT CONFLICT"}),
    pibt_resolutions=len({e["sequence"] for e in physical["events"] if e["event"]=="PIBT RESOLUTION"}),
    lacam_activations=len({e["sequence"] for e in physical["events"] if e["event"]=="RESOLUTION METHOD"}),
    battery_note="Controller's simulated percentage depletion; not measured electrical energy or Wh.",
    robots=robots)
bid_rows={r:dict(deterministic_score=a["deterministic"],predicted_remaining_time_s=(1-a["advice"]["suitability"])*180,
    predicted_exposure=a["advice"]["risk"],ai_adjustment=a["final"]-a["deterministic"],
    final_bid=a["final"],transmitted_rounded_bid=physical["bids"][r]["bid"]) for r,a in physical["audits"].items()}
advised=dict(completed=True,winner=physical["winner"],contacts=physical["collisions"],bids=bid_rows,
    completion_time_sim_s=physical["final"]["time"]-physical["before_bid"]["time"],
    completion_time_wall_s=physical["results"][args.run+"_ADVISED"]["duration"],
    physical_displacement_m=math.dist(physical["final"]["poses"][physical["winner"]][:2],
                                     physical["before_bid"]["poses"][physical["winner"]][:2]),
    final_position=physical["final"]["poses"][physical["winner"]][:2],
    final_task_status="COMPLETED",
    last_telemetry_state=physical["telemetry"][physical["winner"]]["state"],
    telemetry_note="Last telemetry preceded completion; no post-completion IDLE sample was retained in this run.",
    independent_winner_agreement=physical["decisions"])
failsafe={k:v for k,v in fallback.items() if k not in ("physics","telemetry")}
metrics.update(model=dict(schema=SCHEMA,architecture="7 -> 12 embedding; edge-conditioned mean messages; 24 -> 12 -> 2 sigmoid",
    node_features=["x/20","y/16","vx/0.45","vy/0.45","speed/0.45","distance_to_goal/50","task_active"],
    edge_features=["distance/6","relative_speed/0.9","closing_speed/0.9"],
    edge_rule="Directed pair distance <= 6 m, no self edges"),
    cost=cost,evaluation=evaluation,physical_coordination=fleet,physical_ai_bidding=advised,failsafe=failsafe,
    frozen_prediction_evidence=str((directory/"frozen_predictions.csv").relative_to(ROOT)),
    prediction_timing="Version-3 model applied to recorded causal live features at task completion, before adding any run samples to training; not stored online before outcomes.",
    graph_ablation_delta_full_minus_no_neighbor={k:evaluation["fleet"]["full_graph"][k]-evaluation["fleet"]["no_neighbors"][k] for k in evaluation["fleet"]["full_graph"]})
(ROOT/"Webots/edge_ai_metrics.json").write_text(json.dumps(metrics,indent=2))
fl=dict(physical_run=args.run,capture_run=args.capture,status="PASS",physical_rounds=4,
    numerical_verification=rounds,numerical_tolerance=1e-6,
    original_sample_counts_by_round={str(i):next(s["counts"] for s in physical["statuses"] if s["event"]=="AGGREGATED" and s["version"]==i) for i in range(1,5)},
    original_local_losses=metrics["local_training"],replay_local_training=training,
    communication=dict(captured_unique_updates=len(capture["updates"]),allowed_keys=sorted(expected_keys),
        raw_local_experience_in_aggregation_messages=False,encoding="Actual captured UTF-8 JSON payload bytes; excludes DDS/RTPS headers and retransmissions",
        claim="Raw local experience remains local during federated model aggregation.",
        limitations="Normal coordination still shares live pose/intent. No cryptographic privacy, DP, secure aggregation or Byzantine robustness."),
    edge_cost=cost,evaluation=evaluation,
    evidence=dict(capture=str((capture_dir/"capture.json").relative_to(ROOT)),physical=str((directory/"physical_result.json").relative_to(ROOT))))
(ROOT/"Webots/federated_learning_metrics.json").write_text(json.dumps(fl,indent=2))
print(json.dumps(dict(cost=cost,numerical=rounds,physical=fleet,evaluation=evaluation["fleet"]),indent=2))
