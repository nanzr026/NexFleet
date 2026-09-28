"""Summarize measured holdout errors and physical execution without improvement claims."""
import argparse
import hashlib
import json
import math
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
parser=argparse.ArgumentParser()
parser.add_argument("run")
args=parser.parse_args()
directory=ROOT/"Webots/edge_ai_runs"/args.run
physical=json.loads((directory/"physical_result.json").read_text())
assert physical["status"]=="PASS",physical.get("error")
def lines(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line]
evaluations={};training={};models={};graph_stats={}
for rid in ("amr_01","amr_02","amr_03"):
    local=directory/rid
    evaluations[rid]=next(e for e in lines(local/"evaluation.jsonl") if e["version"]==3)
    training[rid]=lines(local/"training.jsonl")
    models[rid]=[json.loads((local/f"model_{i}.json").read_text()) for i in range(1,5)]
    graphs=lines(local/"graphs.jsonl")
    graph_stats[rid]=dict(snapshots=len(graphs),
        edge_counts=sorted({len(g["edges"]) for g in graphs}),
        first_time=graphs[0]["timestamp"],last_time=graphs[-1]["timestamp"])
predictions=[p for e in evaluations.values() for p in e["predictions"]]
mae={key:[sum(abs(p[key][i]-p["target"][i]) for p in predictions)/len(predictions)
          for i in range(2)] for key in ("gnn","no_neighbors","training_mean","kinematic")}
comparison=json.loads((directory/"baseline_comparison.json").read_text())
mae.update(comparison["mae"])
for version in range(4):
    assert models["amr_01"][version]==models["amr_02"][version]==models["amr_03"][version]
parameter_count=sum(len(v) for v in models["amr_01"][0]["weights"].values())
seed=json.loads((ROOT/"Webots/edge_ai_runs/seed/manifest.json").read_text())
deterministic_winner=max(physical["audits"],key=lambda r:physical["audits"][r]["deterministic"])
result=dict(run=args.run,physical_status=physical["status"],
    coordinated_goals=3,advised_task_completed=True,contacts=physical["collisions"],
    minimum_separation_m=physical["minimum_separation"],winner=physical["winner"],
    deterministic_counterfactual_winner=deterministic_winner,
    bids=physical["bids"],bid_audits=physical["audits"],independent_decisions=physical["decisions"],
    parameters=parameter_count,training_sources=seed,local_training=training,
    shared_model_agreement=True,completed_fedavg_rounds=4,
    holdout_samples=len(predictions),holdout_mae=mae,
    remaining_time_mae_seconds={k:v[0]*180 for k,v in mae.items()},
    proximity_exposure_mae={k:v[1] for k,v in mae.items()},
    per_robot_evaluation={r:{k:v for k,v in e.items() if k!="predictions"} for r,e in evaluations.items()},
    graph_statistics=graph_stats,
    limitations=["Fresh repeated C scenario, not an unseen-layout generalization test.",
      "Three seed episodes and one holdout episode; temporal rows are correlated.",
      "Risk is measured proximity exposure, not collision probability.",
      "No matched randomized task-allocation experiment; no fleet-efficiency improvement claim.",
      "No neighbor ablation is an inference-time sensitivity check, not a separately trained architecture.",
      "Fourth round uses local holdout experience only after frozen third-round evaluation."])
result["evidence_sha256"]={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
    for p in directory.rglob("*") if p.is_file()}
(ROOT/"Webots/edge_ai_metrics.json").write_text(json.dumps(result,indent=2))
print(json.dumps({k:result[k] for k in ("physical_status","contacts","minimum_separation_m",
    "completed_fedavg_rounds","holdout_samples","remaining_time_mae_seconds","proximity_exposure_mae")},indent=2))
