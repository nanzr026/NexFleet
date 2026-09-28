"""Standalone plots of measured prediction errors and frozen holdout predictions."""
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2]
metrics=json.loads((ROOT/"Webots/edge_ai_metrics.json").read_text())
directory=ROOT/"Webots/edge_ai_runs"/metrics["run"]
fig=plt.figure(figsize=(14,8),constrained_layout=True)
grid=fig.add_gridspec(2,6)
keys=["gnn","local_only","training_mean","current_proximity","no_neighbors"]
labels=["FedAvg GNN","Local only","Seed mean","Kinematic /\nproximity now","No neighbors"]
colors=["#176b8d","#72a4bd","#aeb9c1","#d4ab68","#a291b2"]
for index,(field,title) in enumerate([
    ("remaining_time_mae_seconds","Remaining completion time: mean absolute error (s)"),
    ("proximity_exposure_mae","Proximity exposure: mean absolute error (0–1)")]):
    ax=fig.add_subplot(grid[0,index*3:(index+1)*3])
    values=[metrics[field][key] for key in keys]
    bars=ax.bar(labels,values,color=colors)
    ax.bar_label(bars,fmt="%.3f",padding=3,fontsize=9)
    ax.set_title(title,fontsize=11);ax.set_ylim(0,max(values)*1.22)
    ax.tick_params(axis="x",labelsize=9)
    ax.grid(axis="y",alpha=.2);ax.set_axisbelow(True)
for index,rid in enumerate(("amr_01","amr_02","amr_03")):
    records=[json.loads(line) for line in (directory/rid/"evaluation.jsonl").read_text().splitlines()]
    report=next(r for r in records if r["version"]==3)
    p=report["predictions"];t0=p[0]["timestamp"]
    ax=fig.add_subplot(grid[1,index*2:(index+1)*2])
    t=[r["timestamp"]-t0 for r in p]
    ax.plot(t,[r["target"][1] for r in p],color="#273846",label="Measured future exposure")
    ax.plot(t,[r["gnn"][1] for r in p],color="#168da3",label="Frozen FedAvg GNN")
    ax.set_title(rid.upper()+" — fresh physical holdout",fontsize=11)
    ax.set_xlabel("Simulation time since first sample (s)")
    ax.set_ylim(-.04,1.04);ax.grid(alpha=.2)
    if index==0:ax.set_ylabel("Near-peer exposure fraction");ax.legend(fontsize=8)
fig.suptitle("Graph Edge-AI: predictive value on one fresh repeated Webots scenario\n"
             "3 coordinated goals + 1 advised task completed; 0 contacts. Risk is proximity exposure, not collision probability.",
             fontsize=13)
fig.savefig(ROOT/"Webots/edge_ai_validation.png",dpi=170)
print(ROOT/"Webots/edge_ai_validation.png")
