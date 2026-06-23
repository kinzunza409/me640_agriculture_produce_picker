#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class BaseLinkWorldAligned(Node):
    def __init__(self):
        super().__init__('base_link_world_aligned')

        # relative names → resolve inside the node's namespace, nothing hardcoded
        self.declare_parameter('odom_topic', 'platform/odom/filtered')
        self.declare_parameter('child_frame', 'base_link_world_aligned')

        odom_topic = self.get_parameter('odom_topic').value
        self.child_frame = self.get_parameter('child_frame').value

        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, odom_topic, self.cb, 10)

    def cb(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp          # copy stamp → no TF extrapolation warnings
        t.header.frame_id = msg.header.frame_id    # parent comes from the msg ('odom'), auto-correct
        t.child_frame_id = self.child_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0            # pinned to ground plane
        t.transform.rotation.w = 1.0               # identity → axes stay world-aligned (no yaw)
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = BaseLinkWorldAligned()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()