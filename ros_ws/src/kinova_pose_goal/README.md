# kinova_pose_goal

Continuous Kinova Gen3 end-effector pose tracking for ROS 2 Jazzy.

The package keeps one upstream `geometry_msgs/msg/PoseStamped` target fixed in
its original TF frame while the Husky moves. It does not perform repeated
point-to-point planning and it does not implement a Cartesian P-controller.
Instead, a small target keeper refreshes the stored message timestamp and sends
the unchanged pose to MoveIt Servo's native POSE mode. MoveIt Servo and the
official Kortex `ros2_control` hardware interface remain responsible for
kinematics, joint limits, singularity and collision handling, smoothing, and
hardware commands.

## Production interface

- Upstream target: `/kinova/target_pose` (`geometry_msgs/msg/PoseStamped`)
- Servo pose command: `/servo_node/pose_target_cmds`
- Enable service: `/kinova_pose_tracker/set_enabled` (`std_srvs/srv/SetBool`)
- Servo status: `/servo_node/status` (`moveit_msgs/msg/ServoStatus`)
- Default planning frame: `base_link`
- Controlled MoveIt tip: `end_effector_link`

Tracking always starts disabled. Starting either launch file does not enable
arm motion.

The upstream target may be published once. While enabled, the target keeper
publishes the same pose and frame at 50 Hz with only `header.stamp` refreshed.
MoveIt Servo uses the latest available TF when transforming that pose into its
IK planning frame.

## Safety behavior

Before enabling, the node requires a stored target, fresh joint states, a valid
latest TF, and the MoveIt Servo pause service. It monitors:

- target TF availability and age;
- per-sample target TF translation and rotation jumps;
- joint-state freshness;
- MoveIt Servo status freshness;
- Servo invalid, collision-halt, singularity-halt, and joint-bound statuses.

On a failure, publishing stops immediately. Servo is allowed to reach its
`incoming_command_timeout` and smooth-halt, then the node calls
`/servo_node/pause_servo`. The fault remains latched. Recovery requires the
configured number of consecutive valid TF samples and an explicit enable
service call. This node logs arm tracking failures; it does not stop the Husky.

Initial discontinuity limits are 0.02 m translation and 0.05 rad rotation per
50 Hz sample. These values must be tuned against the real odometry TF rate and
noise before the 1 m base test.

## Build

For a reproducible hardware workspace, prepare the pinned official Kortex
checkout and verified binary artifacts, then build the required packages:

```bash
cd /project
./scripts/setup_kinova_jazzy_dependencies.sh
./scripts/build_kinova_jazzy.sh
source /project/ros_ws/install_jazzy/setup.bash
```

The setup script refuses to reset or replace an existing mismatched Kortex
checkout. Vendor ZIP files and extracted artifacts remain local and are not
committed.

For a package-only CI build, use a clean Jazzy build base; do not reuse Humble
artifacts:

```bash
cd /project/ros_ws
source /opt/ros/jazzy/setup.bash

colcon --log-base log_target_tracking build \
  --build-base build_target_tracking \
  --install-base install_target_tracking \
  --packages-select kinova_pose_goal

source install_target_tracking/setup.bash
```

## Production launch

First start the official Gen3 7-DoF Kortex/MoveIt launch. Then run:

```bash
ros2 launch kinova_pose_goal servo_tracking.launch.py \
  robot_ip:=192.168.1.10 \
  use_fake_hardware:=false \
  use_internal_bus_gripper_comm:=true
```

The launch selects MoveIt Servo command type `POSE = 2`. After checking the
workspace, controllers, TF, joint state, initial arm posture, and emergency
stop, publish a target and explicitly enable:

```bash
ros2 service call /kinova_pose_tracker/set_enabled \
  std_srvs/srv/SetBool "{data: true}"
```

Disable before changing hardware state or shutting down:

```bash
ros2 service call /kinova_pose_tracker/set_enabled \
  std_srvs/srv/SetBool "{data: false}"
```

## Fake target-frame test

`fake_target_tracking.launch.py` is a test utility, not part of production
odometry. It publishes `fake_target_frame` under `base_link`, captures the
current `end_effector_link` pose in that frame once, and remains stationary.
The motion is a smooth cosine profile: 0.1 m forward in 10 seconds and back in
10 seconds. It performs one round trip and stops.

After fake-hardware robot state and controllers are running:

```bash
ros2 launch kinova_pose_goal fake_target_tracking.launch.py \
  use_fake_hardware:=true
```

Wait for the one-shot target to be stored, enable the tracker explicitly, and
only then start the fake frame:

```bash
ros2 service call /kinova_pose_tracker/set_enabled \
  std_srvs/srv/SetBool "{data: true}"

ros2 service call /fake_target_frame/set_moving \
  std_srvs/srv/SetBool "{data: true}"
```

Calling `set_moving` with `false` pauses the fake-frame trajectory without a TF
jump. Calling it with `true` resumes the same trajectory. Once a full round
trip has completed, a new `true` request starts another cycle from zero.

Do not use the fake target motion on real hardware until the static POSE-mode
test has passed and an operator has approved the exact motion.
