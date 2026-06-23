#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Transform
from rclpy.node import Node
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener


class PidPoseLogger(Node):
    def __init__(self):
        super().__init__('pid_pose_logger')

        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('chassis_frame', 'base_link')
        self.declare_parameter('ee_frame', 'arm_0_end_effector_link')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter(
            'dynamic_pose_topic',
            '/pid_performance/gazebo_dynamic_pose',
        )
        self.declare_parameter(
            'chassis_pose_topic',
            '/pid_performance/chassis_pose',
        )
        self.declare_parameter('ee_pose_topic', '/pid_performance/ee_pose')

        self.world_frame = self._frame_param('world_frame')
        self.chassis_frame = self._frame_param('chassis_frame')
        self.ee_frame = self._frame_param('ee_frame')
        self.dynamic_pose_topic = str(self.get_parameter('dynamic_pose_topic').value)

        publish_rate = max(float(self.get_parameter('publish_rate').value), 0.1)
        self.dynamic_transforms = []
        self.last_warning_time = {}

        self.chassis_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter('chassis_pose_topic').value),
            10,
        )
        self.ee_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter('ee_pose_topic').value),
            10,
        )
        self.create_subscription(
            TFMessage,
            self.dynamic_pose_topic,
            self._dynamic_pose_callback,
            10,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_poses)

        self.get_logger().info(
            'Publishing PID poses from Gazebo dynamic pose topic %s; '
            'chassis=%s, ee=%s, output_frame=%s'
            % (
                self.dynamic_pose_topic,
                self.chassis_frame,
                self.ee_frame,
                self.world_frame,
            )
        )

    def _frame_param(self, name):
        return str(self.get_parameter(name).value).strip().lstrip('/')

    def _dynamic_pose_callback(self, msg):
        self.dynamic_transforms = list(msg.transforms)

    def publish_poses(self):
        chassis_tf = self._find_dynamic_transform(self.chassis_frame)
        if chassis_tf is None:
            self._warn_throttled(
                'chassis_dynamic_pose',
                "No Gazebo dynamic pose found for chassis frame '%s' on %s; "
                'not falling back to 2D odom for PID world-pose output.'
                % (self.chassis_frame, self.dynamic_pose_topic),
            )
            return

        now = self.get_clock().now().to_msg()
        self.chassis_pub.publish(self._pose_from_transform(chassis_tf.transform, now))

        ee_tf = self._find_dynamic_transform(self.ee_frame)
        if ee_tf is not None:
            self.ee_pub.publish(self._pose_from_transform(ee_tf.transform, now))
            return

        try:
            base_to_ee = self.tf_buffer.lookup_transform(
                self.chassis_frame,
                self.ee_frame,
                Time(),
            )
        except TransformException as exc:
            self._warn_throttled(
                'ee_pose',
                "No Gazebo dynamic pose for EE frame '%s' and could not compose "
                'from %s -> %s TF: %s'
                % (self.ee_frame, self.chassis_frame, self.ee_frame, exc),
            )
            return

        world_to_ee = _compose_transforms(chassis_tf.transform, base_to_ee.transform)
        self.ee_pub.publish(self._pose_from_transform(world_to_ee, now))

    def _find_dynamic_transform(self, target_frame):
        target = _normalize_frame(target_frame)
        best = None
        for transform in self.dynamic_transforms:
            child = _normalize_frame(transform.child_frame_id)
            if _frame_matches(child, target):
                if _is_world_like(transform.header.frame_id, self.world_frame):
                    return transform
                best = transform
        return best

    def _pose_from_transform(self, transform, stamp):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.world_frame
        msg.pose.position.x = transform.translation.x
        msg.pose.position.y = transform.translation.y
        msg.pose.position.z = transform.translation.z
        msg.pose.orientation = transform.rotation
        return msg

    def _warn_throttled(self, key, message):
        now = time.monotonic()
        last = self.last_warning_time.get(key, 0.0)
        if now - last >= 5.0:
            self.get_logger().warning(message)
            self.last_warning_time[key] = now


def _normalize_frame(frame):
    return str(frame).strip().strip('/')


def _frame_matches(candidate, target):
    if candidate == target:
        return True
    suffixes = (
        '/' + target,
        '/link/' + target,
        '::' + target,
        '::link::' + target,
    )
    return any(candidate.endswith(suffix) for suffix in suffixes)


def _is_world_like(frame_id, configured_world_frame):
    frame = _normalize_frame(frame_id)
    world = _normalize_frame(configured_world_frame)
    return frame in ('', world, 'world') or frame.endswith('/' + world)


def _compose_transforms(first, second):
    result = Transform()
    rotated = _rotate_vector(first.rotation, second.translation)
    result.translation.x = first.translation.x + rotated[0]
    result.translation.y = first.translation.y + rotated[1]
    result.translation.z = first.translation.z + rotated[2]
    q = _quat_multiply(first.rotation, second.rotation)
    result.rotation.x = q[0]
    result.rotation.y = q[1]
    result.rotation.z = q[2]
    result.rotation.w = q[3]
    return result


def _rotate_vector(q, v):
    qx, qy, qz, qw = _normalize_quat((q.x, q.y, q.z, q.w))
    vx, vy, vz = v.x, v.y, v.z
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def _quat_multiply(a, b):
    ax, ay, az, aw = _normalize_quat((a.x, a.y, a.z, a.w))
    bx, by, bz, bw = _normalize_quat((b.x, b.y, b.z, b.w))
    return _normalize_quat((
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ))


def _normalize_quat(values):
    x, y, z, w = values
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def main(args=None):
    rclpy.init(args=args)
    node = PidPoseLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
