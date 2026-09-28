# BEL warehouse — active Webots R2025a simulation

Webots runs inside Ubuntu 24.04 / WSL through /usr/local/bin/webots.
Gazebo remains a preserved legacy/fallback implementation. Dashboard is outside this migration.

## Worlds and models

- worlds/bel_warehouse.wbt: standalone three-robot wheel-physics milestone. No ROS needed. Each robot drives forward, turns left, turns right, and stops; the observer then pauses the world.
- worlds/bel_warehouse_ros.wbt: the same warehouse with one ROS interface per robot. Robots remain stopped until they receive velocity commands.
- worlds/bel_warehouse_before_native_amrs.wbt: preserved pre-migration world.
- protos/WarehouseRack.proto: open metal posts, beams, three decks, stored pallets/cartons, and component collision geometry.
- protos/Pallet.proto: separate deck boards, runners, support blocks and fork openings.
- protos/WarehouseBox.proto: carton, packing tape, shipping label and barcode detail.
- protos/ChargingDock.proto: platform, rear unit, contacts and indicator.
- protos/AMR.proto: 35 kg chassis, independently driven wheels, encoders, low-friction passive supports, GPS, IMU and real Webots LiDAR.

The floor is 40 m × 32 m; coordinates are ENU (x east, y north, z up). Floor markings have no raised collision surface. Rack footprints, docks, walls, fixed obstacles, pallet positions, robot starts and zones are recorded in config/warehouse_layout.json. The Webots map reads this configuration. Tests check world/map consistency. If you change the layout, update the world and JSON together and run those tests.

All active-world EXTERNPROTO and texture URLs are project-local. Factory asset decisions are in protos/factory/README.md. Existing imported mesh experiments remain untouched.

## Run from Ubuntu / WSL

From ~/BEL_AMR_Fleet:

    # Physical milestone, exits with measured pass/fail results.
    bash tools/webots.sh motion

    # Build the existing ROS package plus Webots adapter entry points.
    bash tools/webots.sh build

    # Terminal 1: Webots ROS world.
    bash tools/webots.sh sim

    # Terminal 2: three navigation nodes and three peer agents.
    bash tools/webots.sh fleet

    # Terminal 3: announce a task and verify the winner's arrival.
    bash tools/webots.sh test-navigation

The launcher defaults to ROS_DOMAIN_ID=26 to isolate the active Webots network from legacy Gazebo bridges. Every participating terminal must use the same domain. No webots_ros2 dependency is assumed or required: Webots Python controllers import installed Jazzy rclpy after the launcher sources /opt/ros/jazzy/setup.bash.

Use only one ROS-enabled Webots world and one fleet launch per domain. Close the previous test world before starting the next. Port 1235 is for Webots controller connections; it is distinct from the ROS domain.

The test-interface command sends direct wheel commands to all three robots. Run it on a fresh ROS world without fleet controllers. The test-safety command follows test-interface and approaches the real rack with AMR03. Do not run direct-command probes alongside navigation controllers. Start a fresh world for test-navigation.

## ROS contract

For each amr_01, amr_02 and amr_03:

| Topic | Type | Meaning |
|---|---|---|
| /model/<id>/cmd_vel | geometry_msgs/Twist | body-forward velocity and yaw rate |
| /model/<id>/odometry | nav_msgs/Odometry | wheel-encoder integration |
| /model/<id>/scan | sensor_msgs/LaserScan | real Webots LiDAR, 360 samples |
| /model/<id>/pose | geometry_msgs/PoseStamped | Webots GPS/IMU world pose |
| /<id>/safety | std_msgs/String | local stop/watchdog reason |
| /<id>/telemetry | std_msgs/String | local navigation status |
| /<id>/task_bid and /<id>/task_claim | std_msgs/String | peer allocation traffic |

The pose frame is explicitly webots_world, replacing Gazebo world_demo for this adapter only. Encoder odometry remains <id>/odom; scans use <id>/lidar. GPS/IMU localization is simulation ground truth, not SLAM. A full TF tree is not published in this milestone.

Wheel commands use radius 0.14 m and separation 0.76 m. Webots left-to-right samples are reversed into the increasing-angle ROS scan convention. Commands older than 0.75 wall-clock seconds stop the motors. Local LiDAR stops translation inside 0.70 m and inhibits close-obstacle turns. Navigation retains the existing 0.80 m stop behavior.

## Reuse and decentralization

webots_nodes.py subclasses the existing controller and peer agent. Base classes expose map/frame/localization hooks; Gazebo defaults remain unchanged.

- Existing AStarPlanner and deterministic peer bidding are reused.
- Every robot has its own planner and P2P agent.
- No central task allocator or fleet movement coordinator is introduced.
- The observer publishes simulation time and records physical poses/images. It never publishes robot commands or allocates tasks.
- Dynamic obstacle insertion and A* replanning remain local.
- Existing Edge-AI, cost learning, FedAvg and federated-agent code is preserved. AI is disabled in the Webots launch during migration validation.

## Evidence and current limits

Reports in logs:

- motion_result.json: independent Supervisor measurements of all three Robot nodes.
- ros_interface_result.json: DDS commands, physical movement, turns, encoders, real scans and watchdog.
- lidar_stop_result.json: AMR03 stops in front of an actual rack.
- navigation_result.json: bids, winner, claim, A* path and physical destination.
- ros_physical_history.jsonl: read-only Supervisor pose records.
- warehouse_ros.png and amr_detail.png: actual Webots-rendered images.

This milestone does not claim a complete SIH system:

- Live multi-robot conflict rerouting/PIBT/LaCAM is not yet verified; existing code is preserved.
- Pallets/cartons are static scene objects. Webots pickup tasks are explicitly rejected; no false carry completion is reported.
- Charging docks are modeled; battery-triggered physical charging/docking is not yet validated.
- Federated training is not connected to a live Webots experience loop.
- No GNN implementation was found in the inspected ROS package; it remains an optional enhancement.
- Dynamic-obstacle rerouting has a planner regression test. The runtime obstacle test verifies LiDAR stopping, not full moving-obstacle rerouting.

## Next physical gate: native load transport

Use an active Webots Connector on the AMR and matching passive Connector on a dynamic pallet with real physical support. Prefer bounded docking tolerances and snap FALSE so attachment does not hide alignment errors. Check presence, request locking, then measure both bodies translating together before delivery. After unlocking, prove separation and pallet stability.

Lock state alone is not evidence of an attached peer. See the [R2025a Connector reference](https://github.com/cyberbotics/webots/blob/R2025a/docs/reference/connector.md). Do not substitute a Supervisor pose-following loop for physical transport.

Advance through conflict-aware rerouting and multi-AMR experiments, local connector carry testing, full delivery/release, and then Edge-AI/FL. Immediate LiDAR safety stays deterministic.
