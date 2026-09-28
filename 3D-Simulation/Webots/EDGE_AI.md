Optional graph advice for the existing Webots fleet
=================================================

The normal webots_fleet.launch.py remains the deterministic baseline. The optional
webots_graph_fleet.launch.py adds three local learners and graph-aware bidder
subclasses. It runs the same webots_navigation executable, physical robot
controllers, A*, reservation validation, PIBT and LaCAM. Neither learner nor bidder
adapter publishes cmd_vel. Availability, battery and load-capability checks run
in the original bidder before the bounded AI adjustment is called.

Graph and targets
-----------------
Each graph has three robot nodes from stamped Webots world-pose DDS messages.
Velocity is a finite difference of observed positions and simulation timestamps.
The seven inputs are x/20, y/16, vx/0.45, vy/0.45, speed/0.45,
distance-to-known-goal/50 and active-task indicator. An idle robot has no goal.
Battery, load, heading and conflict counters are intentionally omitted from this
first model because the immutable seed trajectories do not contain consistent
measurements for all of them. Battery remains part of deterministic bidding.

Directed neighbor edges exist only for measured separation <= 6 m (excluding
self edges). Each carries separation/6, relative speed/0.9 and signed closing
speed/0.9. This is a local-interaction radius, not an assumed communication limit.
Full connectivity can occur when all three robots are physically within 6 m.
Missing, non-finite or stale pose information prevents candidate advice.

A 614-parameter CPU PyTorch network embeds seven inputs into 12 values, forms
neighbor messages from the source embedding and three edge features, averages
messages over actual neighbors, combines own and neighbor embeddings, then
outputs two sigmoid values. No PyTorch Geometric dependency is needed.

Output 1 is normalized remaining completion time (capped at 180 simulation
seconds); suitability = 1 - output 1. Output 2 estimates the time fraction spent
within 2.2 m of another robot over the next 10 simulation seconds, truncated at
own completion. Labels use measured future trajectories with duration weighting.
This is proximity exposure, NOT collision probability. All seed runs are
contact-free, so they cannot support a learned collision-probability claim.

Learning and federation
-----------------------
prepare_graph_data.py exports each robot's own labelled rows from immutable
accepted physical A/B/C reports, with source hashes. Every row includes the
observed fleet context, but each local learner receives only its own labels.
There is no synthetic bootstrap. All three peers start from seed 17 and train
70 Adam epochs at learning rate 0.01 per round. Three initial rounds use only
seed experience.

Each DDS model update contains robot ID, architecture/schema identifier, run ID,
round number, parent model digest, sample count and 614 parameters. Each peer
independently validates the three-member cohort and computes sample-weighted
FedAvg in robot-ID order. There is no aggregation server. Wrong architecture,
stale parent, invalid size/count/values, or conflicting duplicate updates cannot
be applied. Incomplete rounds do not advance the model. Checkpoints use JSON,
not pickle. This is a trusted DDS-domain prototype, not Byzantine-secure FL.

On a fresh physical run, model version 3 remains frozen until all robots finish
their own evaluations. An explicit run-scoped /edge/train_fresh trigger starts
round 4 using each robot's seed buffer plus its new local measured experience.
Evaluation files are written before adding those rows to training. Raw graph
snapshots, local learning losses, checkpoints, advice and physical evidence are
recorded below Webots/edge_ai_runs/<run>/.

Bid integration
---------------
For an eligible robot:
  adjusted = clamp(deterministic + 0.08*(suitability - 0.5) - 0.05*risk, 0, 1)
Each bidder requests advice for the exact task and waits at most 1.5 wall seconds.
Absent, malformed, untrained, task-mismatched or >2-second-old advice returns the
exact deterministic score. An unavailable robot still bids zero. Independent
winner selection, assignment and the execution safety stack remain authoritative.
The adapter logs both deterministic and final scores, so counterfactual rankings
are inspectable. Candidate advice is a transfer from active-task trajectory
training; it is not yet calibrated on a broad dataset of idle candidate choices.

Reproduce from the repository root in Ubuntu WSL
------------------------------------------------
```bash
python3 -m venv --system-site-packages .venv-edge
.venv-edge/bin/python -m pip install -r Webots/requirements-edge.txt
source /opt/ros/jazzy/setup.bash
(cd "ROS 2" && colcon build --symlink-install --packages-select bel_amr_ros)
source "ROS 2/install/setup.bash"
python3 Webots/tests/prepare_graph_data.py
.venv-edge/bin/python -m pytest "ROS 2/src/bel_amr_ros/test/test_graph_learning.py" -q
python3 Webots/tests/run_graph_validation.py --limit 1800
# Use the RUN= identifier printed by the runner:
.venv-edge/bin/python Webots/tests/compare_graph_baselines.py <run>
python3 Webots/tests/export_graph_metrics.py <run>
```

The validation runner uses ROS_DOMAIN_ID=28, starts only its own process groups,
and terminates them on completion/failure. It waits for fresh physical poses and
three matching version-3 digests. It executes the established three-AMR C
coordination scenario, records frozen predictions, verifies the fourth learning
round, then announces a separate task to (7, -10). All three independent winner
decisions, actual bid adjustments and winner displacement are required to pass.
Read-only Supervisor poses and contact events provide physical evidence.

Report error against the seed training mean, unobstructed kinematic time,
current-proximity persistence, a separately trained local-only model (equal
210-epoch local optimization budget), and inference with neighbor messages
removed. A fresh repeated scenario is not an unseen-layout generalization test.
Rows from the same physical episode are correlated. Do not infer fleet throughput
improvement from prediction error or from a single successful advised assignment.


Expanded validation and cost audit
---------------------------------
The completed evidence is in EDGE_AI_FL_VALIDATION.md. To reproduce the DDS
capture and unavailable-inference experiment, use separate runs (the physical
runner has already cleaned up its own processes):
```bash
python3 Webots/tests/capture_federated_rounds.py
python3 Webots/tests/run_graph_failsafe.py
.venv-edge/bin/python Webots/tests/audit_graph_validation.py <physical-run> <capture-run> <failsafe-run>
.venv-edge/bin/python Webots/tests/verify_fourth_graph_round.py
python3 Webots/tests/plot_graph_metrics.py
```
The numerical audit replays seed training and checks independent sample-weighted
sums against original checkpoints. Round-four payloads are explicitly labelled
as offline reconstructions. The failsafe launch sets EDGE_DISABLE_INFERENCE=1;
it launches normal navigation and graph bidders but no inference processes.

Frozen holdout predictions are offline calculations on live recorded causal
features, saved before training on that episode. Do not describe them as
prospectively logged forecasts. The optional graph launch remains experimental:
the measured federated predictor is worse than local-only on this held-out run.
