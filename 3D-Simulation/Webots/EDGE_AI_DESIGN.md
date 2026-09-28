**Advisory graph learning scope**

The deterministic A*/PIBT/LaCAM/reservation/LiDAR stack remains authoritative.
Only an optional Webots bidder extension consumes bounded advisory scores.

Prediction targets:
- Suitability = 1 minus normalized remaining task completion time (cap 180 simulation seconds).
- Coordination risk = fraction of the next ten simulation seconds spent within 2.2 m of another AMR, clipped at task completion. This is a measured proximity-exposure proxy, not collision probability.

Initial training rows come only from accepted physical A/B/C pose histories.
Each robot owns its own labelled rows. No synthetic bootstrap is used.
Snapshots contain measured position and velocity, documented task goals and
active state. Battery, load and heading are omitted from this first model
because the archived trajectories do not reliably contain those measurements.
Live graphs use the same feature schema.

Edges exist only within 6 m and encode measured pair distance, relative speed
and closing speed. The graph feeds a trainable PyTorch message-passing model.
Only model parameters/sample counts cross the federated update topic; no
labelled training rows are exchanged. All peers independently perform
sample-weighted FedAvg with round, architecture and parent-digest checks.

Evaluation uses a fresh physical episode excluded from initial training.
Frozen predictions are compared with training-mean and simple kinematic
baselines; neighbor-message ablation tests whether graph information matters.
Any absence of predictive or operational improvement will be reported.
Fresh local experience is trained only after its frozen-model evaluation.
