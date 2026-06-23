#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from husky_gen3_msgs.msg import TrajectoryPointStamped


class PointTrajectory(Node):
    def __init__(self):
        super().__init__('point_trajectory')

        self.declare_parameter('frame_id',   'arm_0_base_link')
        self.declare_parameter('x',           0.45)
        self.declare_parameter('y',           0.0)
        self.declare_parameter('z',           0.4)
        self.declare_parameter('publish_hz',  5.0)

        hz = self.get_parameter('publish_hz').value

        # relative name → resolves to /<ns>/pid_controller/target under the
        # node's launch namespace
        self.pub = self.create_publisher(
            TrajectoryPointStamped, 'pid_controller/target', 10)

        self.create_timer(1.0 / hz, self._cb)
        self.get_logger().info('PointTrajectory node started')

    def _cb(self):
        msg = TrajectoryPointStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value
        msg.pose.position.x = self.get_parameter('x').value
        msg.pose.position.y = self.get_parameter('y').value
        msg.pose.position.z = self.get_parameter('z').value

        # Point straight down — rotate 180° around x axis
        msg.pose.orientation.x = 1.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 0.0

        # twist left zero-initialised → zero target velocity

        self.pub.publish(msg)
        self.get_logger().info(
            f'Published to {self.pub.topic_name} | '
            f'x: {self.get_parameter("x").value} '
            f'y: {self.get_parameter("y").value} '
            f'z: {self.get_parameter("z").value}',
            throttle_duration_sec=2*60,
        )


def main(args=None):
    rclpy.init(args=args)
    node = PointTrajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()