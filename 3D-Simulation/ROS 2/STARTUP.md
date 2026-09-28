# EdgeFleet runtime startup

From the repository root, source ROS and start one warehouse instance:

```bash
source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH="$PWD/Gazebo Harmonic/models:${GZ_SIM_RESOURCE_PATH:-}"
gz sim -r "Gazebo Harmonic/worlds/openrobotics_warehouse.sdf"
```

In a second terminal:

```bash
cd "ROS 2"
source /opt/ros/jazzy/setup.bash
colcon build --packages-select bel_amr_ros
source install/setup.bash
ros2 launch bel_amr_ros three_amrs.launch.py
```

The fleet launch owns one bridge for all nine odometry/scan/command mappings.
Do not also run the old manual parameter_bridge command. When deliberately
using an externally managed bridge, set `enable_bridge:=false`. A bridge-only
launch is available as `fleet_bridge.launch.py`; do not run it alongside the
default fleet launch. Stop an old fleet launch before starting a replacement.

The controller's `odom_timeout` parameter defaults to 1 second of steady time.
Feedback loss publishes zero velocity, retains the current task and route, and
logs WAITING_FOR_ODOMETRY once. Fresh feedback resumes the preserved route,
subject to the existing CBS-style and LiDAR safety checks. Controllers currently
use wall time; do not enable simulation time without also supplying ROS `/clock`.

Check simulation progress without changing it:

```bash
gz topic -e -t /world/world_demo/stats -n 2
ros2 topic hz /model/amr_01/odometry
ros2 topic hz /model/amr_01/scan
```

To resume an existing paused simulation:

```bash
gz service -s /world/world_demo/control --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean --timeout 5000 --req 'pause: false'
```

Phase 1 validation on 2026-09-21: three watchdog unit tests passed; package
built; isolated real DDS feedback-loss/recovery test preserved task and route;
all three warehouse robots received advancing odometry and scan timestamps.
At the world's 0.01-second physics step, the configured 30 Hz odometry appeared
at 25 Hz in simulation time. Wall-time rates also depend on real-time factor.
This is transport validation, not a collision or throughput result.

For a clean session, the preferred single command is:

```bash
ros2 launch bel_amr_ros digital_twin.launch.py
```

Run it only after stopping existing Gazebo/fleet launches. It starts a headless
warehouse with rendered sensors, releases AMR_02's existing startup pallet joint,
waits for the detach acknowledgement, and then starts the three stacks and one
bridge. The existing pallet attach/detach functionality is retained. Override
`world:=/absolute/path/to/world.sdf` for another checkout or installed layout.
The initializer is specific to this demonstration world's AMR_02 cargo interface.
For a GUI view, connect a Gazebo GUI client to the running server instead of
starting a second server.

Phase 2 measurements in `validation/phase2_original_axes*.json` proved reversed
physical forward/yaw signs before the six joint-axis signs were corrected.
`phase2_corrected_axes*.json` contains post-change physical and odometric poses.
AMR_02 must start detached; an attached remote pallet prevented rotation even
with corrected wheel axes. Measurements include tire/contact slip and are not
an accuracy calibration or evidence of collision-free fleet operation.
