"""Post-run comparison; local-only baselines train on seed data, never holdout labels."""
import argparse
import json
import math
from pathlib import Path
from bel_amr_ros.graph_learning import initial_model, train, predict, weights
ROOT=Path(__file__).resolve().parents[2]
parser=argparse.ArgumentParser();parser.add_argument("run");args=parser.parse_args()
directory=ROOT/"Webots/edge_ai_runs"/args.run
output={}
for rid in ("amr_01","amr_02","amr_03"):
    local=directory/rid
    seed=json.loads((ROOT/f"Webots/edge_ai_runs/seed/{rid}.json").read_text())
    # Equal local optimization budget: three rounds of 70 epochs, without FedAvg.
    model=initial_model();training=train(model,seed,epochs=210)
    (local/"local_only_model.json").write_text(json.dumps(weights(model)))
    experiences=[json.loads(line) for line in (local/"local_experience.jsonl").read_text().splitlines()]
    heldout=next(e for e in experiences if e["task_id"]==args.run+"_"+rid)["rows"]
    errors=[]
    for row in heldout:
        g=row["graph"];own=g["nodes"][rid]
        nearest=min(math.hypot(own["x"]-v["x"],own["y"]-v["y"])
                    for r,v in g["nodes"].items() if r!=rid)
        local_prediction=predict(model,g,rid)
        persistence=[min(1.,own["distance_to_goal"]/.45/180),float(nearest<2.2)]
        errors.append(dict(target=row["target"],local_only=local_prediction,
                           current_proximity=persistence))
    output[rid]=dict(training=training,samples=len(errors),predictions=errors)
all_rows=[r for v in output.values() for r in v["predictions"]]
mae={k:[sum(abs(r[k][i]-r["target"][i]) for r in all_rows)/len(all_rows) for i in range(2)]
     for k in ("local_only","current_proximity")}
(directory/"baseline_comparison.json").write_text(json.dumps(dict(per_robot=output,mae=mae),indent=2))
print(json.dumps(mae,indent=2))
