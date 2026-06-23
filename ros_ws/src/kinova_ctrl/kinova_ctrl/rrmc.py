#!/usr/bin/env python3
import numpy as np
import pinocchio as pin

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from husky_gen3_msgs.msg import TrajectoryPointStamped

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped transform support

NUM_JOINTS = 7


class RRMController(Node):
    """Resolved-rate motion control for a 7-DOF Kinova Gen3 on a Husky base.

    Control law (all textbook): closed-loop IK (CLIK) + damped least squares (DLS)
    + null-space posture bias — i.e. Whitney resolved-rate motion control with
    Nakamura/Wampler damping and Liégeois gradient-projection redundancy resolution.

    Key design choices:
      - ONE resolved-rate step per cycle (no solve-to-convergence). The step scales
        with the Cartesian error, so the command self-stabilizes: as the EE reaches
        the target it stops moving on its own.
      - The step is seeded from the previous COMMAND, never the measured state, so
        command and measurement can't chase each other (the old jitter mechanism).
        Measured joint_states is used ONLY to seed the command on first enable.
      - The target is held in a world frame (odom) and re-transformed into the arm
        base frame EVERY cycle. As the base drives, world->base changes, so the
        target's pose in the arm frame moves and the loop tracks it — that is how
        the EE holds a world-fixed pose while the base teleops underneath.
      - Consumer is the joint_trajectory_controller TOPIC interface, position-only,
        with time_from_start = one control period so points chain instead of being
        preempted before they land.
    """

    def __init__(self):
        super().__init__('rrm_controller')

        # ── Parameters ──────────────────────────────────────────────────────────
        self.declare_parameter('kp_cart', 8.0)      # Cartesian error gain [1/s] (~0.25s time const)
        self.declare_parameter('ns_gain', 0.2)      # null-space pull toward q_rest
        self.declare_parameter('damping', 1e-6)     # DLS damping λ
        self.declare_parameter('ctrl_freq', 50.0)   # Hz
        self.declare_parameter('publish_deadband', 1e-3)   # rad; below this, don't republish
        self.declare_parameter('urdf_path', '/opt/ros/humble/share/kortex_description/robots/gen3_2f85.urdf')  # generic Gen3
        self.declare_parameter('arm_base_frame', 'arm_0_base_link')
        self.declare_parameter('ee_frame', 'gen3_end_effector_link')
        self.declare_parameter('world_frame', 'odom')   # frame the target is held in


        
        self.publish_deadband = self.get_parameter('publish_deadband').value
        self._last_published = None

        # test_case: True  -> generate a fixed target in world_frame, ignore the subscriber.
        #            False -> take targets from the subscriber.
        self.declare_parameter('test_case', False)
        # Fixed test target [x, y, z, qx, qy, qz, qw] in world_frame (test_case only).
        self.declare_parameter('test_target_pose', [0.5, 0.0, 0.8, 1.0, 0.0, 0.0, 0.0])

        self.KP_CART = self.get_parameter('kp_cart').value
        self.NS_GAIN = self.get_parameter('ns_gain').value
        self.DAMP    = self.get_parameter('damping').value
        self.dt      = 1.0 / self.get_parameter('ctrl_freq').value
        self.arm_base_frame = self.get_parameter('arm_base_frame').value
        self.world_frame    = self.get_parameter('world_frame').value
        self.test_case      = self.get_parameter('test_case').value

        # use_sim_time is auto-declared by rclpy; read+log so the active clock is explicit.
        # Set it at launch (-p use_sim_time:=true) in Gazebo — the control timer uses
        # the node clock, which follows this parameter.
        self.get_logger().info(f'use_sim_time = {self.get_parameter("use_sim_time").value}')

        # ── Pinocchio model ──────────────────────────────────────────────────────
        # Full URDF includes the gripper; lock everything except the 7 arm joints so
        # the reduced model is a clean 7-DOF arm. Joints 1,3,5,7 are continuous
        # (JointModelRUBZ, nq=2, encoded as cos/sin); 2,4,6 are revolute (nq=1).
        # Total nq=11, nv=7 — hence the angle<->config helpers below.
        urdf_path = self.get_parameter('urdf_path').value
        full_model = pin.buildModelFromUrdf(urdf_path)

        arm_joints = [f'gen3_joint_{i}' for i in range(1, 8)]
        locked = [
            full_model.getJointId(name)
            for name in full_model.names[1:]        # skip 'universe'
            if name not in arm_joints
        ]
        q_ref = pin.neutral(full_model)             # gripper frozen at neutral pose
        self.pin_model = pin.buildReducedModel(full_model, locked, q_ref)
        self.pin_data  = self.pin_model.createData()

        ee_name = self.get_parameter('ee_frame').value
        if not self.pin_model.existFrame(ee_name):
            raise RuntimeError(f'EE frame {ee_name!r} not in model after reduction')
        self.ee_frame_id = self.pin_model.getFrameId(ee_name)

        q_rest_angles = np.array([0.0, 0.26, 0.0, -2.27, 0.0, -0.96, 1.57])   # mid-range home, radians
        self.q_rest_config = self._angles_to_config(q_rest_angles)

        self.joint_name_map = {f'gen3_joint_{i}': f'arm_0_joint_{i}' for i in range(1, 8)}
        self.joint_names    = list(self.joint_name_map)

        # ── State ──────────────────────────────────────────────────────────────
        self.q = self.qdot = None             # measured (used ONLY to seed the command)
        self.q_cmd     = None                 # running command, Pinocchio nq-layout
        self.theta_cmd = None                 # running command, 7 angles
        self.target_pose  = None              # geometry_msgs/Pose, expressed in target_frame
        self.target_frame = None              # frame_id the target pose lives in

        # ── Transforms ───────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── I/O ──────────────────────────────────────────────────────────────────
        self.create_subscription(JointState, 'platform/joint_states', self._joint_state_cb, 10)
        self.cmd_pub = self.create_publisher(
            JointTrajectory, 'arm_0_joint_trajectory_controller/joint_trajectory', 10)

        if self.test_case:
            # Self-generated fixed target in world_frame; subscriber is not created.
            p = self.get_parameter('test_target_pose').value
            self.target_pose  = self._pose_from_list(p)
            self.target_frame = self.world_frame
            self.get_logger().warn(
                f'TEST MODE: holding fixed target {list(p)} in {self.world_frame!r}; '
                'subscriber ignored. Set test_target_pose to a reachable pose for your setup.')
        else:
            self.create_subscription(
                TrajectoryPointStamped, 'pid_controller/target', self._target_cb, 10)

        # ── Control timer ──────────────────────────────────────────────────────
        self.create_timer(self.dt, self._control_loop_cb)

        self._publishing = False
        self.get_logger().info('RRMController started')

    # ── Callbacks ───────────────────────────────────────────────────────────────

    def _joint_state_cb(self, msg: JointState):
        self.q, self.qdot = self._unpack_joint_state(msg, self.joint_names, self.joint_name_map)
        if self.q_cmd is None:                  # first measurement seeds the command, once
            self.theta_cmd = self.q.copy()
            self.q_cmd     = self._angles_to_config(self.q)

            # one-shot FK sanity check: where does the model think the EE is?
            pin.forwardKinematics(self.pin_model, self.pin_data, self.q_cmd)
            pin.updateFramePlacement(self.pin_model, self.pin_data, self.ee_frame_id)
            self.get_logger().info(
                f'FK EE in model base: {self.pin_data.oMf[self.ee_frame_id].translation}')

    def _target_cb(self, msg: TrajectoryPointStamped):
        # Store the raw pose + its frame; it is re-transformed every cycle in the
        # control loop, so a world-frame target stays world-fixed while the base
        # drives. Position-only — the twist field is ignored.
        self.target_pose  = msg.pose
        self.target_frame = msg.header.frame_id

    def _control_loop_cb(self):
        # Gate: need a seeded command (first joint_state) and a target.
        if self.q_cmd is None or self.target_pose is None:
            return

        # Re-transform the target into the arm base frame EVERY cycle — this is the
        # base-coordination mechanism (see class docstring).
        X_des = self._target_in_base()
        if X_des is None:
            return    # TF not ready this cycle; skip publishing

        if not self._publishing:
            self.get_logger().info('Starting to publish joint commands')
            self._publishing = True

        theta_cmd = self._resolved_rate_step(X_des)
        if (self._last_published is None
                or np.max(np.abs(theta_cmd - self._last_published)) >= self.publish_deadband):
            self._last_published = theta_cmd
            controller_joint_names = [self.joint_name_map[n] for n in self.joint_names]
            self.cmd_pub.publish(self._pack_joint_trajectory(theta_cmd, controller_joint_names))

    # ── Core ──────────────────────────────────────────────────────────────────────

    def _target_in_base(self) -> pin.SE3:
        """Transform the stored target pose (in target_frame) into the arm base
        frame using the LATEST available TF. Returns None if TF isn't ready."""
        ps = PoseStamped()
        ps.header.frame_id = self.target_frame
        ps.header.stamp = rclpy.time.Time().to_msg()   # 0 = use latest available transform
        ps.pose = self.target_pose
        try:
            pose_in_base = self.tf_buffer.transform(
                ps, self.arm_base_frame,
                timeout=rclpy.duration.Duration(seconds=0.05))
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'TF transform failed: {e}', throttle_duration_sec=1.0)
            return None
        return self._pose_to_se3(pose_in_base.pose)

    def _resolved_rate_step(self, X_des: pin.SE3) -> np.ndarray:
        """One resolved-rate motion-control step (CLIK + DLS + null-space posture bias).

        Seeds from the COMMAND (self.q_cmd), never the measured state, so command and
        measurement can't chase each other. Self-stabilizing: the step scales with the
        Cartesian error, so as the EE reaches X_des the command stops moving.

        Frame conventions (easy to get wrong on this model):
          - X_des is in the arm base frame (= Pinocchio model origin).
          - Error and position Jacobian are in the EE LOCAL frame; the Jlog6 term
            corrects the Jacobian for the log-map linearisation (standard CLIK form).
        """
        q = self.q_cmd

        pin.forwardKinematics(self.pin_model, self.pin_data, q)
        # EE is a frame, not a joint, so update its placement to read oMf[ee]
        pin.updateFramePlacement(self.pin_model, self.pin_data, self.ee_frame_id)

        # Transform from current EE frame to desired EE frame (LOCAL)
        iMd = self.pin_data.oMf[self.ee_frame_id].actInv(X_des)
        err = pin.log(iMd).vector                       # 6D error twist in the EE local frame

        self.get_logger().info(
            f'|err| pos={np.linalg.norm(err[:3]):.3f}  rot={np.linalg.norm(err[3:]):.3f}',
            throttle_duration_sec=0.5)

        # Position Jacobian in the EE LOCAL frame, matching the error frame
        J = pin.computeFrameJacobian(self.pin_model, self.pin_data, q,
                                     self.ee_frame_id, pin.ReferenceFrame.LOCAL)
        # Correct the Jacobian for the log-map linearisation (standard Pinocchio CLIK form)
        J = -pin.Jlog6(iMd.inverse()).dot(J)

        # Primary task: damped least-squares pseudo-inverse of the 6×nv Jacobian
        Jpinv = J.T.dot(np.linalg.inv(J.dot(J.T) + self.DAMP * np.eye(6)))

        # Null-space projector biases the redundant DOF toward q_rest so the elbow
        # doesn't drift — a gentle one-step pull, not a re-converged posture.
        N = np.eye(self.pin_model.nv) - Jpinv.dot(J)
        dq_rest = pin.difference(self.pin_model, q, self.q_rest_config)   # q → q_rest in tangent space

        # One step: primary Cartesian error reduction + secondary posture bias,
        # integrated on the manifold.
        v = Jpinv.dot(-self.KP_CART * err) + N.dot(self.NS_GAIN * dq_rest)
        self.q_cmd = pin.integrate(self.pin_model, q, v * self.dt)

        self.theta_cmd = self._config_to_angles(self.q_cmd, self.theta_cmd)
        return self.theta_cmd

    def _angles_to_config(self, theta: np.ndarray) -> np.ndarray:
        """Map 7 joint angles into Pinocchio's nq-vector, encoding continuous
        joints (nq=2) as (cos, sin)."""
        q = np.zeros(self.pin_model.nq)
        for j, th in enumerate(theta):                 # j: 0..6 over arm joints
            joint = self.pin_model.joints[j + 1]       # +1 to skip 'universe'
            idx, nq = joint.idx_q, joint.nq
            if nq == 1:
                q[idx] = th
            else:                                       # nq == 2, continuous
                q[idx]     = np.cos(th)
                q[idx + 1] = np.sin(th)
        return q

    def _config_to_angles(self, q: np.ndarray, theta_ref: np.ndarray) -> np.ndarray:
        theta = np.zeros(7)
        for j in range(7):
            joint = self.pin_model.joints[j + 1]
            idx, nq = joint.idx_q, joint.nq
            if nq == 1:
                theta[j] = q[idx]
            else:
                a = np.arctan2(q[idx + 1], q[idx])
                # unwrap to the revolution nearest the reference (previous command)
                theta[j] = theta_ref[j] + np.arctan2(np.sin(a - theta_ref[j]), np.cos(a - theta_ref[j]))
        return theta

    # ── Helpers ───────────────────────────────────────────────────────────────────

    @staticmethod
    def _unpack_joint_state(msg: JointState, joint_names: list[str], name_map: dict) -> tuple[np.ndarray, np.ndarray]:
        remapped = [name_map.get(n, n) for n in joint_names]
        order = [msg.name.index(n) for n in remapped]
        q    = np.array([msg.position[i] for i in order])
        qdot = np.array([msg.velocity[i] for i in order]) if msg.velocity else np.zeros(len(order))
        return q, qdot

    def _pack_joint_trajectory(self, theta_cmd: np.ndarray, joint_names: list[str]) -> JointTrajectory:
        """Single-point JointTrajectory for the JTC topic interface, position-only.
        time_from_start = one control period so consecutive points chain smoothly
        instead of each one being preempted before it lands. (If you see stutter
        under load, give a little headroom with ~1.5–2× self.dt.)"""
        msg = JointTrajectory()
        msg.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = theta_cmd.tolist()
        point.time_from_start.sec     = 0
        point.time_from_start.nanosec = int(self.dt * 1e9)   # 20 ms @ 50 Hz

        msg.points = [point]
        return msg

    @staticmethod
    def _pose_from_list(p) -> Pose:
        """Build a geometry_msgs/Pose from [x, y, z, qx, qy, qz, qw]."""
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = float(p[0]), float(p[1]), float(p[2])
        pose.orientation.x = float(p[3])
        pose.orientation.y = float(p[4])
        pose.orientation.z = float(p[5])
        pose.orientation.w = float(p[6])
        return pose

    @staticmethod
    def _pose_to_se3(pose: Pose) -> pin.SE3:
        """Convert a geometry_msgs/Pose to a Pinocchio SE3 transform."""
        translation = np.array([pose.position.x, pose.position.y, pose.position.z])
        q = pose.orientation
        rotation = pin.Quaternion(q.w, q.x, q.y, q.z).toRotationMatrix()
        return pin.SE3(rotation, translation)


def main(args=None):
    rclpy.init(args=args)
    node = RRMController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()