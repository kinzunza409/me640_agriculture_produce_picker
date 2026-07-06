import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class StraightDriveTest(Node):
    """Publish a short, conservative forward Twist command for Husky checks."""

    def __init__(self):
        super().__init__('straight_drive_test')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('speed', 0.1)
        self.declare_parameter('duration', 2.0)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('ramp_time', 0.5)
        self.declare_parameter('dry_run', False)

        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.speed = float(self.get_parameter('speed').value)
        self.duration = float(self.get_parameter('duration').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.ramp_time = float(self.get_parameter('ramp_time').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)

        self._validate_parameters()
        self.publisher = None
        if not self.dry_run:
            self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)

    def _validate_parameters(self):
        if not self.cmd_vel_topic:
            raise ValueError('cmd_vel_topic must not be empty')
        if self.duration < 0.0:
            raise ValueError('duration must be non-negative')
        if self.publish_rate <= 0.0:
            raise ValueError('publish_rate must be greater than zero')
        if self.ramp_time < 0.0:
            raise ValueError('ramp_time must be non-negative')
        if abs(self.speed) > 0.1:
            self.get_logger().warn(
                'Requested speed is above the conservative 0.1 m/s first-test limit.'
            )

    def run(self):
        period = 1.0 / self.publish_rate
        planned_messages = max(1, int(math.ceil(self.duration * self.publish_rate)))

        self.get_logger().warn(
            'Hardware safety: lift the wheels or clear the path, confirm e-stop state, '
            'and verify the cmd_vel topic before running on a real Husky.'
        )
        self.get_logger().info(
            f'cmd_vel_topic={self.cmd_vel_topic}, speed={self.speed:.3f} m/s, '
            f'duration={self.duration:.3f} s, publish_rate={self.publish_rate:.3f} Hz, '
            f'ramp_time={self.ramp_time:.3f} s, dry_run={self.dry_run}'
        )

        if self.duration == 0.0:
            self.get_logger().info('Duration is zero; sending only stop commands.')
            self.publish_stop()
            return

        start_time = time.monotonic()
        for _ in range(planned_messages):
            if not rclpy.ok():
                break

            elapsed = time.monotonic() - start_time
            if elapsed >= self.duration:
                break

            command = Twist()
            command.linear.x = self.speed * self._ramp_scale(elapsed)
            self._publish_or_log(command, elapsed)

            rclpy.spin_once(self, timeout_sec=0.0)
            sleep_time = period - (time.monotonic() - start_time - elapsed)
            if sleep_time > 0.0:
                time.sleep(sleep_time)

        self.publish_stop()

    def _ramp_scale(self, elapsed):
        if self.ramp_time == 0.0:
            return 1.0
        return min(1.0, max(0.0, elapsed / self.ramp_time))

    def _publish_or_log(self, command, elapsed):
        if self.dry_run:
            self.get_logger().info(
                f'[dry-run] t={elapsed:.2f}s would publish '
                f'linear.x={command.linear.x:.3f}, angular.z={command.angular.z:.3f} '
                f'to {self.cmd_vel_topic}'
            )
            return

        self.publisher.publish(command)

    def publish_stop(self):
        stop_command = Twist()
        stop_count = max(5, int(math.ceil(0.5 * self.publish_rate)))
        period = 1.0 / self.publish_rate

        for index in range(stop_count):
            if self.dry_run:
                self.get_logger().info(
                    f'[dry-run] stop {index + 1}/{stop_count}: would publish zero Twist '
                    f'to {self.cmd_vel_topic}'
                )
            else:
                self.publisher.publish(stop_command)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

        self.get_logger().info('Straight drive test complete; zero Twist command sent.')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StraightDriveTest()
        node.run()
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().warn('Interrupted; publishing zero Twist before exit.')
            node.publish_stop()
    except Exception as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f'straight_drive_test failed: {exc}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
