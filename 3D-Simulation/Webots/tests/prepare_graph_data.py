"""Export per-AMR experience from immutable accepted physical evidence."""
import hashlib
import json
from pathlib import Path
from run_coordination import scenario
from bel_amr_ros.fleet_graph import IDS
from bel_amr_ros.graph_experience import seed_samples
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/"Webots/edge_ai_runs/seed"
OUT.mkdir(parents=True,exist_ok=True)
samples={r:[] for r in IDS}; sources={}
for name in "ABC":
    path=ROOT/f"Webots/logs/accepted/{name}/coordination_{name}_result.json"
    report=json.loads(path.read_text())
    nodes,edges,starts,goals,yaws=scenario(name)
    target={IDS[i]:nodes[g] for i,g in enumerate(goals)}
    sources[name]=dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    for rid in IDS:
        samples[rid].extend(seed_samples(report,target,rid))
for rid,rows in samples.items():
    (OUT/f"{rid}.json").write_text(json.dumps(rows))
manifest=dict(sources=sources,counts={r:len(s) for r,s in samples.items()},
              label="remaining time / 180; measured near-peer exposure under 2.2m over next 10s")
(OUT/"manifest.json").write_text(json.dumps(manifest,indent=2))
print(json.dumps(manifest,indent=2))
