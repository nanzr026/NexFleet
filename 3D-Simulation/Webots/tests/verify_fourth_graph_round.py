"""Reconstruct unrecorded round-four update payloads from preserved local experience."""
import json
import math
import time
from pathlib import Path
from bel_amr_ros.fleet_graph import IDS,digest
from bel_amr_ros.graph_learning import initial_model,load,train,weights,ARCH
ROOT=Path(__file__).resolve().parents[2]
fl=json.loads((ROOT/"Webots/federated_learning_metrics.json").read_text())
directory=ROOT/"Webots/edge_ai_runs"/fl["physical_run"]
base=json.loads((directory/"amr_01/model_3.json").read_text())["weights"]
updates=[];timings={}
for rid in IDS:
    samples=json.loads((ROOT/f"Webots/edge_ai_runs/seed/{rid}.json").read_text())
    records=[json.loads(s) for s in (directory/rid/"local_experience.jsonl").read_text().splitlines()]
    samples+=next(r["rows"] for r in records if r["task_id"]==fl["physical_run"]+"_"+rid)
    model=initial_model();load(model,base)
    timings[rid]=train(model,samples)
    updates.append(dict(run=fl["physical_run"],robot_id=rid,version=4,architecture=ARCH,
                        parent=digest(base),samples=len(samples),weights=weights(model)))
count=sum(u["samples"] for u in updates)
actual=json.loads((directory/"amr_01/model_4.json").read_text())["weights"]
difference=max(abs(math.fsum(u["samples"]*u["weights"][k][i] for u in updates)/count-actual[k][i])
               for k in actual for i in range(len(actual[k])))
assert difference<1e-6,difference
report=dict(round=4,source="Offline deterministic reconstruction, not original DDS capture",
    max_absolute_parameter_difference=difference,
    sample_counts={u["robot_id"]:u["samples"] for u in updates},
    reconstructed_update_payload_bytes={u["robot_id"]:len(json.dumps(u).encode()) for u in updates},
    total_reconstructed_payload_bytes=sum(len(json.dumps(u).encode()) for u in updates),
    reconstruction_training=timings)
fl["fourth_round_reconstruction"]=report
(ROOT/"Webots/federated_learning_metrics.json").write_text(json.dumps(fl,indent=2))
(directory/"round4_reconstructed_updates.json").write_text(json.dumps(updates))
print(json.dumps(report,indent=2))
