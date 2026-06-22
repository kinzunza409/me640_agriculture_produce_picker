#!/usr/bin/env python3
import numpy as np
import pinocchio as pin

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped transform support

NUM_JOINTS = 7


class PIDController(Node):

    def __init__(self):
        super().__init__('pid_controller')

        self.declare_parameter('kp', [1.0] * NUM_JOINTS)
        self.declare_parameter('kd', [0.1] * NUM_JOINTS)
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('arm_base_frame', 'base_link')
        self.declare_parameter('ee_frame', 'end_effector_link')

        self.Kp = np.diag(self.get_parameter('kp').value)
        self.Kd = np.diag(self.get_parameter('kd').value)

        # Pinocchio model — built in __init__, used in _solve_ik
        urdf_path = self.get_parameter('urdf_path').value
        self.pin_model, _, _ = pin.buildModelsFromUrdf(urdf_path)
        self.pin_data = self.pin_model.createData()
        self.ee_frame_id = self.pin_model.getFrameId(
            self.get_parameter('ee_frame').value)

        # State
        self.q    = np.zeros(NUM_JOINTS)
        self.qdot = np.zeros(NUM_JOINTS)

        # TF
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.create_subscription(JointState,   '~/joint_states', self._js_cb,   10)
        self.create_subscription(PoseStamped,  '~/target_pose',  self._pose_cb, 10)

        # Publisher — /a200_0000/joint_trajectory_controller/joint_trajectory
        # Verify topic name with: ros2 topic list | grep trajectory
        self.cmd_pub = self.create_publisher(
            JointTrajectory,
            'joint_trajectory_controller/joint_trajectory',
            10,
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _js_cb(self, msg: JointState):
        pass

    def _pose_cb(self, msg: PoseStamped):
        pass

    # ── Core ──────────────────────────────────────────────────────────────────

    def _transform_to_base(self, pose: PoseStamped) -> PoseStamped:
        """Transform an incoming PoseStamped into the arm base frame."""
        pass

    def _solve_ik(self, target_pos: pin.SE3, target_vel: pin.Motion, q0: np.ndarray, qdot0: np.ndarray):
        """
        Closed-loop IK (CLIK) via damped least-squares.
        Returns (q_d, qdot_d) or (None, None) on failure.
        """
        EPS    = 1e-4
        IT_MAX = 1000
        DT     = 1e-1
        DAMP   = 1e-12

        q = q0.copy()

        for _ in range(IT_MAX):
            pin.forwardKinematics(self.pin_model, self.pin_data, q)

            # Transform from current EE frame to desired EE frame
            iMd = self.pin_data.oMi[self.ee_frame_id].actInv(target_pos)

            # Log map gives us the 6D error twist in the joint frame
            err = pin.log(iMd).vector

            if np.linalg.norm(err) < EPS:
                # Compute Jacobian at converged q
                J = pin.computeJointJacobian(self.pin_model, self.pin_data, q, self.ee_frame_id)

                # Correct Jacobian for the log map linearisation around the error
                J = -np.dot(pin.Jlog6(iMd.inverse()), J)

                # Map target Cartesian velocity to joint velocity via damped pseudo-inverse
                q_d    = q
                qdot_d = J.T.dot(np.linalg.solve(J.dot(J.T) + DAMP * np.eye(6), target_vel.vector))
                return q_d, qdot_d

            J = pin.computeJointJacobian(self.pin_model, self.pin_data, q, self.ee_frame_id)
            J = -np.dot(pin.Jlog6(iMd.inverse()), J)

            # Damped least-squares step drives q toward target_pos
            v = -J.T.dot(np.linalg.solve(J.dot(J.T) + DAMP * np.eye(6), err))
            q = pin.integrate(self.pin_model, q, v * DT)

        self.get_logger().warn(
            f'IK did not converge (final error {np.linalg.norm(err):.4f})',
            throttle_duration_sec=1.0,
        )
        return None, None

    def _compute_control(self, q_des, qdot_des) -> np.ndarray:
        """PD law: Kp*(q_des - q) + Kd*(qdot_des - qdot). Returns (NUM_JOINTS,)."""
        pass

    def _publish(self, u: np.ndarray):
        """Pack control output into a JointTrajectory and publish."""
        pass

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _unpack_joint_state(msg: JointState, joint_names: list[str]) -> tuple[np.ndarray, np.ndarray]:
        """Unpack a JointState message into q and qdot arrays, sorted to match joint_names order."""
        order = [msg.name.index(n) for n in joint_names]
        q    = np.array([msg.position[i] for i in order])
        qdot = np.array([msg.velocity[i] for i in order]) if msg.velocity else np.zeros(len(order))
        return q, qdot

    @staticmethod
    def _pose_to_se3(pose: PoseStamped) -> pin.SE3:
        """Convert a PoseStamped message to a Pinocchio SE3 transform."""
        p = pose.pose.position
        q = pose.pose.orientation
        translation = np.array([p.x, p.y, p.z])
        rotation    = pin.Quaternion(q.w, q.x, q.y, q.z).toRotationMatrix()
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