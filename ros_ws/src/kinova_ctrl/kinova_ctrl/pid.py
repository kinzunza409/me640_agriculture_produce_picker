#!/usr/bin/env python3
import numpy as np
import pinocchio as pin

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped, Twist
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from husky_gen3_msgs.msg import TrajectoryPointStamped

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped transform support

NUM_JOINTS = 7


class PIDController(Node):

    def __init__(self):
        super().__init__('pid_controller')
        
        self.declare_parameter('kp', [1.0] * NUM_JOINTS)
        self.declare_parameter('kd', [0.1] * NUM_JOINTS)
        self.declare_parameter('urdf_path', '/opt/ros/humble/share/kortex_description/robots/gen3_2f85.urdf') # generic Gen3
        self.declare_parameter('arm_base_frame', 'arm_0_base_link')
        self.declare_parameter('ee_frame', 'gen3_end_effector_link')
        self.declare_parameter('ctrl_freq', 10.0)  # Hz
        

        self.Kp = np.diag(self.get_parameter('kp').value)
        self.Kd = np.diag(self.get_parameter('kd').value)

        # Pinocchio model — full URDF includes the gripper (nq=21); lock everything
        # except the 7 arm joints so the reduced model is a clean 7-DOF arm (nq=7)
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

        self.get_logger().info(
            'Reduced model joints: ' +
            str([(self.pin_model.names[i + 1], j.shortname(), j.nq, j.nv)
                for i, j in enumerate(self.pin_model.joints[1:])]))
        self.pin_data  = self.pin_model.createData()

        ee_name = self.get_parameter('ee_frame').value
        if not self.pin_model.existFrame(ee_name):
            raise RuntimeError(f'EE frame {ee_name!r} not in model after reduction')
        self.ee_frame_id = self.pin_model.getFrameId(ee_name)

        q_rest_angles = np.array([0.0, 0.26, 0.0, -2.27, 0.0, -0.96, 1.57])   # mid-range home, radians
        self.q_rest_config = self._angles_to_config(q_rest_angles)

        self.joint_name_map = {f'gen3_joint_{i}': f'arm_0_joint_{i}' for i in range(1, 8)}
        self.joint_names    = list(self.joint_name_map)

        # State
        self.q    = None
        self.qdot = None
        self.target_pos = None
        self.target_vel = None

        # Transforms
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.arm_base_frame = self.get_parameter('arm_base_frame').value

        # Subscribers
        self.create_subscription(JointState,   'platform/joint_states', self._joint_state_cb,   10)
        self.create_subscription(TrajectoryPointStamped,  'pid_controller/target',  self._target_cb, 10)

        # Timers
        self.create_timer(1.0 / self.get_parameter('ctrl_freq').value, self._control_loop_cb)

        # Publisher
        self.cmd_pub = self.create_publisher(JointTrajectory, 'arm_0_joint_trajectory_controller/joint_trajectory', 10)

        self._publishing = False
        self.get_logger().info('PIDController started')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _joint_state_cb(self, msg: JointState):
        self.q, self.qdot = self._unpack_joint_state(msg, self.joint_names, self.joint_name_map)
    

    def _target_cb(self, msg: TrajectoryPointStamped):
        self.target_pos, self.target_vel = self._transform_to_base(msg)

    def _control_loop_cb(self):
        if all(v is not None for v in (self.q, self.qdot, self.target_pos, self.target_vel)):
            # if this is the first time running
            if not self._publishing:
                self.get_logger().info('Starting to publish joint commands')
                self._publishing = True

            # get desired state from IK
            q_d, qdot_d = self._solve_ik(self.target_pos, self.target_vel, self.q, self.qdot)
            if q_d is None:
                return   # IK failed this cycle; skip publishing

            # push desired state to arm and let internal PID handle it for now
            controller_joint_names = [self.joint_name_map[n] for n in self.joint_names]
            msg = self._pack_joint_trajectory(q_d, qdot_d, controller_joint_names)
            self.cmd_pub.publish(msg)


    # ── Core ──────────────────────────────────────────────────────────────────

    def _transform_to_base(self, msg: TrajectoryPointStamped) -> tuple[pin.SE3, pin.Motion]:
        """Transform a TrajectoryPointStamped into SE3 and Motion in the arm base frame."""
        # Wrap pose in a PoseStamped so tf2 can transform it
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.header.stamp = rclpy.time.Time().to_msg()   # 0 = use latest available transform
        pose_stamped.pose   = msg.pose

        try:
            pose_in_base = self.tf_buffer.transform(
                pose_stamped, self.arm_base_frame,
                timeout=rclpy.duration.Duration(seconds=0.05))
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'TF transform failed: {e}', throttle_duration_sec=1.0)
            return None, None

        # Twist doesn't have a stamped tf2 transformer, rotate it manually
        try:
            tf = self.tf_buffer.lookup_transform(
                self.arm_base_frame, msg.header.frame_id,
                rclpy.time.Time())
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=1.0)
            return None, None

        R = pin.Quaternion(
            tf.transform.rotation.w,
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
        ).toRotationMatrix()

        twist_in_base        = Twist()
        lin                  = np.array([msg.twist.linear.x,  msg.twist.linear.y,  msg.twist.linear.z])
        ang                  = np.array([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])
        rot_lin              = R @ lin
        rot_ang              = R @ ang
        twist_in_base.linear.x  = rot_lin[0]; twist_in_base.linear.y  = rot_lin[1]; twist_in_base.linear.z  = rot_lin[2]
        twist_in_base.angular.x = rot_ang[0]; twist_in_base.angular.y = rot_ang[1]; twist_in_base.angular.z = rot_ang[2]

        return self._pose_to_se3(pose_in_base.pose), self._twist_to_motion(twist_in_base)

    def _solve_ik(self, target_pos: pin.SE3, target_vel: pin.Motion, q0: np.ndarray, qdot0: np.ndarray):
        """
        Closed-loop IK (CLIK) via damped least-squares with null-space posture bias.
        Frame conventions (easy to get wrong on this model):
          - target_pos is in the arm base frame (= Pinocchio model origin).
          - Position error is in the EE LOCAL frame; position Jacobian and Jlog6 are LOCAL.
          - target_vel is in base-frame axes, so the velocity Jacobian is LOCAL_WORLD_ALIGNED.
        The 7-DOF arm is redundant for a 6-DOF target, so a null-space term biases the
        redundant elbow toward q_rest to keep the solution repeatable cycle-to-cycle.
        Returns (q_d, qdot_d) as 7-element vectors, or (None, None) if position fails to converge.
        """
        EPS    = 1e-4
        IT_MAX = 1000     # enough iterations for DLS to fully descend at DT=0.1
        DT     = 0.1      # larger values overshoot and the loop oscillates
        DAMP   = 1e-6
        NS_GAIN = 0.7     # null-space pull toward q_rest; lower if convergence slows

        # Seed from measured arm state, encoded into nq-layout (continuous joints as cos/sin)
        q = self._angles_to_config(q0)

        err = np.full(6, np.inf)   # defined for the post-loop warning

        for _ in range(IT_MAX):
            pin.forwardKinematics(self.pin_model, self.pin_data, q)
            # EE is a frame, not a joint, so update its placement to read oMf[ee]
            pin.updateFramePlacement(self.pin_model, self.pin_data, self.ee_frame_id)

            # Transform from current EE frame to desired EE frame (LOCAL)
            iMd = self.pin_data.oMf[self.ee_frame_id].actInv(target_pos)

            # 6D error twist in the EE local frame
            err = pin.log(iMd).vector

            if np.linalg.norm(err) < EPS:
                # Converged: solve the velocity mapping
                # target_vel is in base-frame axes, so use a base-aligned frame Jacobian
                J_vel = pin.computeFrameJacobian(
                    self.pin_model, self.pin_data, q, self.ee_frame_id,
                    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

                # Damped least-squares pseudo-inverse: qdot = J^T (JJ^T + λI)^-1 v
                q_d    = self._config_to_angles(q, q0)
                qdot_d = J_vel.T.dot(
                    np.linalg.solve(J_vel.dot(J_vel.T) + DAMP * np.eye(6),
                                    target_vel.vector))
                return q_d, qdot_d

            # Not converged: take a DLS step toward target_pos
            # Position Jacobian in the EE LOCAL frame, matching the error frame
            J = pin.computeFrameJacobian(
                self.pin_model, self.pin_data, q, self.ee_frame_id,
                pin.ReferenceFrame.LOCAL)

            # Correct the Jacobian for the log-map linearisation (standard Pinocchio CLIK form)
            J = -pin.Jlog6(iMd.inverse()).dot(J)

            # Primary task: damped least-squares pseudo-inverse of the 6×nv Jacobian
            J_pinv = J.T.dot(np.linalg.inv(J.dot(J.T) + DAMP * np.eye(6)))

            # Null-space projector biases redundant DOF toward q_rest so the elbow doesn't drift
            N = np.eye(self.pin_model.nv) - J_pinv.dot(J)
            dq_rest = pin.difference(self.pin_model, q, self.q_rest_config)   # q → q_rest in tangent space

            # Combined step: primary error reduction + secondary posture bias, integrated on the manifold
            v = -J_pinv.dot(err) + N.dot(NS_GAIN * dq_rest)
            q = pin.integrate(self.pin_model, q, v * DT)

        # Position loop exhausted without converging — caller skips publishing
        self.get_logger().warn(
            f'IK did not converge (final error {np.linalg.norm(err):.4f})',
            throttle_duration_sec=1.0)
        return None, None

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
                # unwrap to the revolution nearest the measured angle
                theta[j] = theta_ref[j] + np.arctan2(np.sin(a - theta_ref[j]), np.cos(a - theta_ref[j]))
        return theta

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _unpack_joint_state(msg: JointState, joint_names: list[str], name_map: dict) -> tuple[np.ndarray, np.ndarray]:
        remapped = [name_map.get(n, n) for n in joint_names]
        order = [msg.name.index(n) for n in remapped]
        q    = np.array([msg.position[i] for i in order])
        qdot = np.array([msg.velocity[i] for i in order]) if msg.velocity else np.zeros(len(order))
        return q, qdot
    
    @staticmethod
    def _pack_joint_trajectory(q_d: np.ndarray, qdot_d: np.ndarray, joint_names: list[str]) -> JointTrajectory:
        """Pack q_d and qdot_d into a single-point JointTrajectory."""
        msg = JointTrajectory()
        msg.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions  = q_d.tolist()
        point.velocities = qdot_d.tolist()
        point.time_from_start.sec     = 0
        point.time_from_start.nanosec = 100_000_000 # 

        msg.points = [point]
        return msg

    @staticmethod
    def _twist_to_motion(twist: Twist) -> pin.Motion:
        """Convert a geometry_msgs/Twist to a Pinocchio Motion."""
        linear  = np.array([twist.linear.x,  twist.linear.y,  twist.linear.z])
        angular = np.array([twist.angular.x, twist.angular.y, twist.angular.z])
        return pin.Motion(linear, angular)

    @staticmethod
    def _pose_to_se3(pose: Pose) -> pin.SE3:
        """Convert a geometry_msgs/Pose to a Pinocchio SE3 transform."""
        translation = np.array([pose.position.x, pose.position.y, pose.position.z])
        q = pose.orientation
        rotation = pin.Quaternion(q.w, q.x, q.y, q.z).toRotationMatrix()
        return pin.SE3(rotation, translation)


def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()