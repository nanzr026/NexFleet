# Live decentralized Webots coordination

This extension preserves the Robot motor/encoder/LiDAR interface, individual
A* planner and P2P bidding implementation. Five files containing committed merge
markers were restored from e7e8f54 with user authorization; other incoming files
were retained.

## Architecture

A navigate task can opt into a named coordination session with its peer
membership and a small lane graph. The issuer specifies goals and traversable
geometry, never motion decisions or a winning robot. Each WebotsNavigation
adapter independently:

1. Computes its original A* route.
2. Broadcasts its start, goal, priority, route, graph digest and reservation via
   its own /amr_0x/reservation topic (std_msgs/String JSON over ROS 2 DDS).
3. Predicts unsafe separation on time-resampled A* routes.
4. Runs recursive PIBT priority inheritance and assignment backtracking.
   Non-goal waits are ranked after safe moving alternatives.
5. Broadcasts a deterministic successor digest. Movement requires peer agreement.
6. Uses measured arrival barriers and a current-peer segment clearance check.
   Stale peer data or disagreement inhibits movement.
7. Detects repeated joint configurations; after four stationary simulation seconds
   reports suspected deadlock and after eight reports confirmation.
8. Runs bounded LaCAM only after the local PIBT process cannot make progress.
   The search uses configuration DFS, lazy partial-constraint BFS, and PIBT
   successor generation. Failure holds robots stopped.
9. Executes only its own segment through the existing motion controller.
   Original scan stopping and the Robot-level LiDAR motor override remain active.

No FleetCoordinator was introduced. The test probe sends goals and measures
results; the Supervisor observer only reads physical positions and contacts.
Graph edges must pass the existing A* occupancy line-of-sight check. A session
requires initial and goal poses at graph vertices. This is a bounded lane-graph
coordination mode, not unrestricted continuous-space multi-robot navigation.
Tasks without coordination metadata retain the previous individual A* behavior.

## Tests

Run from the repository root in Ubuntu, with Webots R2025a installed:

    source /opt/ros/jazzy/setup.bash
    source "ROS 2/install/setup.bash"
    bash tools/webots.sh build
    python3 Webots/tests/run_baseline.py
    python3 Webots/tests/run_coordination.py A
    python3 Webots/tests/run_coordination.py B
    python3 Webots/tests/run_coordination.py C

The runner starts a fresh generated scenario world and the existing six-node
fleet launch in ROS domain 27. It closes only its own processes. The baseline
runner uses domain 26. Do not run another instance in the same domain.

A is an open crossing, B an opposing aisle with a passing bay behind the
higher-priority robot, and C a simultaneous three-robot crossing/swap.
All robot movement after initialization is through physical wheel motors.

## Evidence and limitations

Reports are in Webots/logs/coordination_[A-C]_result.json. They include per-agent
events, raw Supervisor position samples, completion and metrics. Collision count
is the number of episodes containing non-floor contacts above z=0.04 m, checked
on every 16 ms physics step, together with minimum physical center separation.
This contact criterion is specific to the flat-floor test worlds.

The first development run accidentally retained the independent execution path;
it recorded one collision. Its evidence is preserved in
failed_independent_execution_physics.jsonl. A later wait-based coordinated run
passed with zero contacts, archived as development_wait_A_result.json. Neither
is substituted for the final reroute-first acceptance tests.

LaCAM algorithm reference:
https://kei18.github.io/lacam-project/
The implemented search is bounded; no claim of completeness on timeout or of
LaCAM* optimality is made. Simulated tests do not certify real-world safety.
