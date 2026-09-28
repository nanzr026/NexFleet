# Edge-AI / Federated Learning Validation

The optional AI/FL pipeline works physically, but the current FedAvg model is **not an accuracy improvement over local-only learning** on this held-out episode. Graph messages help exposure prediction relative to message removal, while worsening remaining-time error. AI remains advisory and disabled in the normal baseline launch.

## UNIT TESTED

**46 tests passed:** all 38 accepted coordination/workflow/learning regressions plus 8 graph-learning tests. Coverage includes measured inputs, distance edges, message influence, disconnected-node isolation, local training, weighted aggregation, invalid/stale updates and invalid/NaN advice fallback. Log: Webots/edge_ai_runs/regressions.log.

Accepted A*/PIBT/LaCAM/LiDAR runtime files still match the runtime manifest. No Dashboard or Gazebo changes were made.

## OFFLINE EVALUATED

The same **157 held-out rows** were used for all comparisons. Model version 3 was trained only on accepted A/B/C seed trajectories. A fresh repeated C episode supplied evaluation data; this is not an unseen-layout test. Rows within an episode are correlated.

Predictions were computed at completion from recorded causal live graph snapshots, before those samples entered training. They were **not logged prospectively before outcomes occurred**. The model never received future labels as inputs. Frozen predictions and actual values are preserved in Webots/edge_ai_runs/graph_1790144140293694551/frozen_predictions.csv.

Local-only models used each AMR’s seed labels and 210 local epochs, matching the local epoch budget of three 70-epoch FedAvg rounds. These comparison models were trained offline; the physical run used the federated model.

| Predictor | Time MAE (s) | Time RMSE (s) | Exposure MAE | Exposure RMSE |
|---|---:|---:|---:|---:|
| Kinematic time / current proximity | 24.526 | 28.619 | 0.1890 | 0.3448 |
| Local-only GNN | 7.147 | 10.034 | 0.1005 | 0.1440 |
| Federated GNN, full graph | 16.579 | 21.577 | 0.2200 | 0.2685 |
| Same federated GNN, no messages | 14.520 | 17.433 | 0.4276 | 0.5035 |

| AMR | Local time MAE (s) | Federated time MAE (s) | Local exposure MAE | Federated exposure MAE |
|---|---:|---:|---:|---:|
| amr_01 | 5.315 | 15.341 | 0.1523 | 0.2266 |
| amr_02 | 10.537 | 16.498 | 0.1253 | 0.1606 |
| amr_03 | 5.522 | 17.901 | 0.0233 | 0.2740 |

Full graph minus no-neighbor MAE: **+2.059 s** for remaining time (worse), **-0.2076** for exposure (better). The simple current-proximity predictor also beats the federated GNN on exposure. No improved generalization or fleet-efficiency claim is justified. Message removal is an inference-time ablation, not a separately trained architecture.

Exposure is the measured future fraction of time within 2.2 m of a peer over at most 10 simulation seconds, not collision probability. Suitability is one minus remaining time normalized by 180 seconds. Candidate-task suitability transfers from active coordination trajectories and remains uncalibrated.

## FEDERATED TRAINING VALIDATED

Three AMRs independently trained and aggregated over DDS. Rounds 1–3 used seed samples only. Round 4 used each robot’s new local experience after its frozen evaluation was saved. All peers produced matching models.

A seed-only DDS replay captured actual model-update payloads. An independent float64 scalar weighted sum was compared with saved float32 global parameters without calling the implementation aggregator. Replay models exactly match the original physical-run models.

| Round | AMR01 samples | AMR02 samples | AMR03 samples | Max parameter error | Total update JSON bytes |
|---|---:|---:|---:|---:|---:|
| 1 | 259 | 259 | 55 | 2.975e-08 | 40490 |
| 2 | 259 | 259 | 55 | 4.587e-08 | 40400 |
| 3 | 259 | 259 | 55 | 5.107e-08 | 40352 |
| 4 (reconstructed) | 311 | 312 | 107 | 5.683e-08 | 40321 |

Round-4 updates were reconstructed from preserved seed and new local buffers; its byte counts are not packet-capture measurements. Captured rounds 1–3 contain only parameters, sample counts, robot/run IDs, architecture, round and parent digest. Byte counts exclude DDS headers, retransmissions and multicast delivery overhead.

**Raw local experience remains local during federated model aggregation.** Ordinary coordination still shares live poses and intentions. No cryptographic privacy, differential privacy, secure aggregation or Byzantine guarantees are implemented.

The model has **614 parameters**, 2456 raw float32 bytes and a **13287-byte JSON checkpoint**. Over 300 warmed calls, graph preparation plus inference averaged **0.842 ms**, with **1.888 ms p95**, using one PyTorch CPU thread in WSL. This is not an embedded-device benchmark.

Local training duration in the three-peer DDS replay:

| AMR | Round 1 (s) | Round 2 (s) | Round 3 (s) |
|---|---:|---:|---:|
| amr_01 | 13.004 | 0.721 | 0.713 |
| amr_02 | 12.891 | 0.735 | 0.746 |
| amr_03 | 12.607 | 0.660 | 0.544 |

The first call includes optimizer initialization. Aggregator calls averaged 10.822 ms (round 1), 5.913 ms (round 2), 3.870 ms (round 3); p95 values are in the FL JSON. Training cost is from the instrumented replay, not retroactively measured original-run duration.

## PHYSICALLY EXECUTED IN WEBOTS

Run: graph_1790144140293694551. Supervisor positions and non-floor contacts establish **3/3 coordinated goals, 0 contacts, 0 permanent deadlocks**, a **58.592 simulation-second makespan** and **1.865 m minimum separation**. There were 3 distinct predicted conflicts, 5 joint PIBT resolutions and 0 LaCAM activations. The accepted B run remains the physical LaCAM evidence.

| AMR | Completion (sim s) | Path (m) | Average speed (m/s) | Waiting (sim s) | Replans |
|---|---:|---:|---:|---:|---:|
| amr_01 | 58.592 | 3.865 | 0.0660 | 44.400 | 0 |
| amr_02 | 58.528 | 7.478 | 0.1278 | 17.824 | 1 |
| amr_03 | 58.528 | 7.605 | 0.1299 | 20.704 | 1 |

Each controller reported about 5.7 simulated battery percentage points consumed, not measured electrical energy. Wall durations and per-AMR conflict/resolution counts are retained in JSON.

After evaluation and round 4, the decentralized task to (7, -10) used these scores:

| AMR | Deterministic | Predicted time (s) | Exposure | AI adjustment | Final score |
|---|---:|---:|---:|---:|---:|
| amr_02 | 0.74291 | 28.289 | 0.6343 | -0.00429 | 0.73862 |
| amr_03 | 0.62629 | 23.482 | 0.3933 | +0.00990 | 0.63619 |
| amr_01 | 0.87291 | 18.823 | 0.3838 | +0.01245 | 0.88536 |

All three selected **AMR01**, which physically moved **2.939 m** and completed with **0 contacts** in **9.664 simulation seconds** from the announcement-phase snapshot (36.044 controller wall seconds). The deterministic counterfactual selects the same winner: advice changed scores, not this allocation.

The task result is COMPLETED and the final physical position meets tolerance. The last retained telemetry still says MOVING_TO_GOAL because it predates completion; this run did not retain a later IDLE sample.

Controlled failsafe run: failsafe_1790163213691340072. No inference processes were launched. All bidders logged **AI_FAILURE** and used exact deterministic scores. AMR01 physically travelled **3.601 m**, completed with **0 contacts**, and reached confirmed **IDLE**. Time from pre-announcement snapshot to final observation: **12.736 simulation seconds**. This single-task test required no PIBT/LaCAM activation; their authoritative code remains unchanged.

The first AI-run attempt rejected assignments on stale startup localization, with no movement or contact. Failure evidence is retained. The retry added a fresh-pose readiness gate and retries of unacknowledged test assignments; controller freshness limits were not weakened.

## Evidence and reproduction

- Webots/edge_ai_metrics.json: graph, evaluations, physical metrics, bids and failsafe.
- Webots/federated_learning_metrics.json: rounds, numerical verification, communication and costs.
- Webots/edge_ai_validation.png: error comparisons and exposure traces.
- Webots/edge_ai_runs/: raw graphs, local buffers, frozen predictions, checkpoints, DDS capture and physics.
- Webots/EDGE_AI.md: architecture and reproduction commands.

Previous Webots/coordination_metrics.json, Webots/COORDINATION_VALIDATION.md and accepted A/B/C evidence remain separate and intact.
