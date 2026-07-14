# kinova_pose_goal

Continuous Kinova Gen3 end-effector pose tracking for ROS 2 Jazzy.

The node stores the latest `geometry_msgs/msg/PoseStamped` goal. At 50 Hz it
uses the latest TF to compare that goal with `end_effector_link`, then publishes a
bounded Cartesian velocity command to MoveIt Servo. This allows the arm to
keep tracking a world/odom target while the Husky base moves.

The official Kinova and MoveIt packages remain responsible for the hardware
interface, ros2_control, collision checking, singularity handling, and joint
trajectory output.

## Interface

- Target: `/kinova/target_pose` (`geometry_msgs/msg/PoseStamped`)
- Servo output: `/servo_node/delta_twist_cmds`
- Enable service: `/kinova_pose_tracker/set_enabled` (`std_srvs/srv/SetBool`)
- Planning frame: `base_link`
- Tool frame: `end_effector_link`

Tracking starts disabled. Starting this node does not command motion.

## Build

```bash
cd /project/ros_ws
source /opt/ros/jazzy/setup.bash
source install_jazzy/setup.bash
colcon --log-base log_jazzy build \
  --build-base build_jazzy \
  --install-base install_jazzy \
  --packages-select kinova_pose_goal
```

## Runtime sequence

1. Start the official Gen3 7-DoF hardware/MoveIt launch.
2. Launch `servo_tracking.launch.py` with the same robot description arguments.
3. The launch selects MoveIt Servo's Twist command type automatically.
4. Publish a target pose.
5. Enable tracking through `/kinova_pose_tracker/set_enabled` only after the
   target, TF tree, controller state, workspace, and emergency stop are checked.

The initial command limits are 0.03 m/s translation and 0.10 rad/s rotation.
