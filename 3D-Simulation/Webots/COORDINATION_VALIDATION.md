**Physical Webots coordination validation — completed**

All final acceptance scenarios passed. Test A remains preserved; Test B naturally activated LaCAM after repeated PIBT configurations and a bounded stall; Test C completed all three goals without LaCAM. The accepted baseline and its 38 targeted regression tests remain intact.

| Test | Goals | Contacts | Minimum separation | PIBT steps | LaCAM episodes | Resolved deadlocks | Makespan |
|---|---:|---:|---:|---:|---:|---:|---:|
| A | 2/2 | 0 | 1.864 m | 4 | 0 | 0 | 41.104 s |
| B | 2/2 | 0 | 1.860 m | 4 | 1 | 1 | 173.008 s |
| C | 3/3 | 0 | 1.863 m | 5 | 0 | 0 | 59.280 s |

Permanent deadlocks: zero in every final scenario. Times are simulation seconds. Fleet counts deduplicate independently logged peer observations. In B, one fallback episode involved two independent LaCAM invocations.

**Physical execution**

- A: crossing A* routes predicted unsafe separation. AMR01 received higher priority; AMR02 physically took a side detour. Both finished.
- B: opposing routes used a single-lane graph through the rack aisle with a passing bay. PIBT first advanced AMR01, then made it retreat. Configurations [1, 2] and [0, 1] were each visited twice. An 8.048-second no-progress hold followed that repeated-state condition. Both agents confirmed deadlock at simulation time 80.48 s, independently searched 18 lazy-constraint expansions, and agreed on [[0,1],[1,2],[0,2],[3,1],[3,0],[0,4],[1,4],[2,4]]. LaCAM was not manually triggered.
- C: three simultaneous crossing/swap routes produced three distinct predicted pair conflicts. All agents agreed on reservations. AMR02 and AMR03 physically detoured; all three finished. LaCAM did not activate.

Test B physical LaCAM verification checked all 7 joint steps against Supervisor poses at subsequent reservation boundaries. Maximum measured arrival error was 0.1388 m (acceptance tolerance 0.25 m). Movement after activation: amr_01: 10.947 m, amr_02: 7.467 m.

**Architecture**

Three Webots Robot/rclpy interfaces retain wheel control, encoder odometry and the LiDAR motor override. Three navigation adapters independently compute A*, publish std_msgs/String DDS reservations, predict conflicts, run recursive PIBT with backtracking, and agree on successor digests. Measured-arrival barriers and current-peer segment clearance checks accommodate unequal motion speeds. Repeated configurations plus a bounded hold invoke local copies of bounded LaCAM. Each adapter commands only its own robot through the existing controller. Three existing P2P bidders remain separate. No FleetCoordinator was introduced; the Supervisor only observes physics.

**Regression results**

38 targeted tests passed: the original 27 plus 11 coordination tests. Physical reruns passed forward/left/right/stop, ROS cmd_vel, encoder odometry, 360-sample scans, command watchdog, rack approach LiDAR stopping, A* navigation, and independent three-agent bidding. LiDAR stop drift was 0.000761 m. Working A*, bidding and the Robot interface were not rewritten.

**Files changed**

Restored exactly from e7e8f54, with user authorization, because HEAD contained committed merge markers:
- ROS 2/src/bel_amr_ros/bel_amr_ros/p2p_agent.py
- ROS 2/src/bel_amr_ros/bel_amr_ros/robot_controller.py
- ROS 2/src/bel_amr_ros/bel_amr_ros/warehouse_map.py
- ROS 2/src/bel_amr_ros/launch/three_amrs.launch.py
- ROS 2/src/bel_amr_ros/setup.py

Modified for this phase:
- ROS 2/src/bel_amr_ros/bel_amr_ros/webots_nodes.py — coordination extension.
- Webots/tests/verify_navigation.py — longer startup/runtime allowances.
- Webots/tests/verify_ros_interface.py — longer startup allowance; physical assertions unchanged.

Created:
- ROS 2/src/bel_amr_ros/bel_amr_ros/coordination_search.py
- ROS 2/src/bel_amr_ros/bel_amr_ros/webots_coordination.py
- ROS 2/src/bel_amr_ros/test/test_coordination.py
- Webots/controllers/coordination_observer/coordination_observer.py
- Webots/tests/run_baseline.py
- Webots/tests/run_coordination.py
- Webots/tests/run_coordination_suite.py
- Webots/tests/export_coordination_metrics.py
- Webots/tests/plot_coordination.py
- Webots/COORDINATION.md
- Webots/COORDINATION_VALIDATION.md
- Webots/coordination_metrics.json
- Webots/coordination_validation.png
- Generated worlds under Webots/worlds: coordination_A.wbt, coordination_B.wbt, coordination_C.wbt, regression_motion.wbt, regression_ros.wbt. Saved user warehouse worlds were preserved.

**Commands**

Run in Ubuntu from the repository root. Avoid another simulator in ROS domain 26 or 27.

```bash
source /opt/ros/jazzy/setup.bash
source "ROS 2/install/setup.bash"
bash tools/webots.sh build
python3 -m pytest "ROS 2/src/bel_amr_ros/test/test_coordination.py" "ROS 2/src/bel_amr_ros/test/test_webots.py" "ROS 2/src/bel_amr_ros/test/test_workflow.py" "ROS 2/src/bel_amr_ros/test/test_learning.py" -q
python3 Webots/tests/run_baseline.py
python3 Webots/tests/run_coordination_suite.py
python3 Webots/tests/export_coordination_metrics.py
python3 Webots/tests/plot_coordination.py
```

The completed run used the suite for A/B, then reran only C after correcting test readiness. C initially received a task before AMR03 had localized to its scenario pose; rejection held the other robots stopped. The probe now requires all three telemetry poses to match their scenario starts. No coordination algorithm changed for this fix.

**Evidence and metrics**

Webots/coordination_metrics.json contains start/goal/final positions, completion time, measured path length, average speed, waiting time, replans and final state for every active robot. Fleet metrics include makespan, distance, mean speed, waiting, distinct conflicts, PIBT steps, LaCAM episodes and agent invocations, deadlock detections, minimum separation, contacts and completed goals. Completion includes reservation barriers; waiting includes holding after reaching an individual goal.

Raw reports include complete pose histories and events: Webots/logs/coordination_A_result.json, coordination_B_result.json and coordination_C_result.json. Accepted copies, fleet/world logs, individual metrics and SHA-256 manifests are preserved separately under Webots/logs/accepted/A, B and C. The runtime source manifest is Webots/logs/coordination_build_manifest.json. All runtime source hashes still match; only the test readiness check changed after A/B acceptance.

Non-floor contact episodes above z=0.04 m were checked every 16 ms physics step, not inferred from planner geometry. This criterion applies to these flat-floor scenarios. Minimum separation comes from independent physical positions.

**Development failures and limits**

An early development run accidentally retained independent execution because the adapter extension had not applied. It recorded one contact episode. Evidence is preserved in Webots/logs/failed_independent_execution_physics.jsonl and distinguished from accepted runs in the metrics JSON. Pre-task capability/readiness checks now guard against this setup error. The first C readiness failure is preserved separately and recorded zero contacts.

Validated scope: explicit coordination-session membership and a small statically validated lane graph for up to three robots. Ordinary tasks retain individual A* behavior. Arbitrary continuous-space missions, moving external obstacles, hardware deployment and physical network-failure recovery are not validated. Stale/mismatched reservations have unit coverage. LiDAR rack stopping is physically verified; it is not claimed as a standalone guarantee against all peer collisions. Bounded LaCAM stops on failure; no optimality or timeout completeness claim is made. Pallet transport, Dashboard, Edge-AI and federated learning were outside this phase.

Algorithm reference: https://kei18.github.io/lacam-project/
