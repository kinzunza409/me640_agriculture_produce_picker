#!/usr/bin/env python3
"""
ROS 2 node that connects to a single Shimmer3 IMU via pyshimmer (Bluetooth SPP,
LogAndStream firmware) and publishes raw accelerometer/gyroscope data as
sensor_msgs/Imu.

No orientation fusion is done here by design — orientation_covariance[0] is set
to -1 (no estimate), and a separate downstream node is expected to do fusion.

The following was verified directly against the actually-installed pyshimmer
1.0.0 source (pip package, not the `main` branch on GitHub — main is mid-refactor
per its own CHANGELOG and is NOT API-compatible with 1.0.0; ESensorGroup's import
path and the gyro channel names below differ between the two):
  - DataPacket is dict-like (pkt[EChannelType.X]); a channel absent from the
    current stream configuration raises KeyError.
  - EChannelType.ACCEL_LN_X/Y/Z are correct. The gyro channels are named
    GYRO_MPU9150_X/Y/Z in 1.0.0 (NOT GYRO_X/Y/Z, which only exists on main).
  - ESensorGroup is NOT exported from top-level `pyshimmer` in 1.0.0 — import
    it from pyshimmer.dev.channels directly.
  - ShimmerBluetooth.set_sensors(Iterable[ESensorGroup]) must be called before
    start_streaming(), or the requested channels won't be in the packet at
    all — start_streaming() derives the packet format from get_inquiry(),
    which only returns currently-enabled sensors. ACCEL_LN and GYRO must be
    explicitly enabled.
  - ShimmerBluetooth.set_sampling_rate(sr: float) exists and is used to set
    the rate at connect time.
  - Streamed accel/gyro values are RAW, UNCALIBRATED ADC COUNTS. pyshimmer's
    streaming path does no unit conversion (only battery voltage gets a
    calibration helper, calibrate_u12_adc_value — accel/gyro do not).
    Calibration coefficients (offset bias, sensitivity, alignment matrix) are
    available via get_all_calibration() but nothing applies them; that's a
    deliberate follow-up, not done here. Until then this node publishes raw
    counts in the linear_acceleration/angular_velocity fields and marks
    linear_acceleration_covariance[0] / angular_velocity_covariance[0] = -1 to
    flag that these aren't yet trustworthy physical quantities.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from serial import Serial
from pyshimmer import ShimmerBluetooth, DEFAULT_BAUDRATE, DataPacket, EChannelType
from pyshimmer.dev.channels import ESensorGroup


class ShimmerImuNode(Node):
    def __init__(self):
        super().__init__('shimmer_imu_node')

        self.declare_parameter('serial_port', '/dev/rfcomm0')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('topic_name', 'imu/data')
        self.declare_parameter('sampling_rate_hz', 51.2)

        self.serial_port = self.get_parameter('serial_port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.topic_name = self.get_parameter('topic_name').value
        self.sampling_rate_hz = self.get_parameter('sampling_rate_hz').value

        # Relative topic name -> respects any namespace the node is launched under.
        self.pub = self.create_publisher(Imu, self.topic_name, qos_profile_sensor_data)

        self.shim_dev = None
        self._connect_and_start()

    def _connect_and_start(self):
        try:
            ser = Serial(self.serial_port, DEFAULT_BAUDRATE)
            self.shim_dev = ShimmerBluetooth(ser)
            self.shim_dev.initialize()
            self.get_logger().info(f"Connected to Shimmer on {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Shimmer on {self.serial_port}: {e}")
            raise

        try:
            self.shim_dev.set_sensors([ESensorGroup.ACCEL_LN, ESensorGroup.GYRO])
            self.get_logger().info("Enabled ACCEL_LN and GYRO sensor groups")
        except Exception as e:
            self.get_logger().error(
                f"Failed to enable ACCEL_LN/GYRO sensors: {e}. Streaming will likely "
                f"KeyError on every packet since those channels won't be active."
            )

        try:
            self.shim_dev.set_sampling_rate(self.sampling_rate_hz)
            self.get_logger().info(f"Set Shimmer sampling rate to {self.sampling_rate_hz} Hz")
        except Exception as e:
            self.get_logger().error(f"Failed to set sampling rate, continuing with device default: {e}")

        self.get_logger().warn(
            "Publishing RAW, UNCALIBRATED accel/gyro ADC counts, NOT m/s^2 / rad/s. "
            "Calibration is not yet implemented (see module docstring). "
            "linear_acceleration_covariance[0] and angular_velocity_covariance[0] are "
            "set to -1 to mark these fields as not yet meaningful physical quantities."
        )

        self.shim_dev.add_stream_callback(self._on_packet)
        self.shim_dev.start_streaming()
        self.get_logger().info(
            f"Streaming started, publishing to '{self.topic_name}' with frame_id '{self.frame_id}'"
        )

    def _on_packet(self, packet: DataPacket):
        # DataPacket is dict-like: packet[EChannelType.X]. A channel that isn't
        # present in the packet raises KeyError (confirmed in pyshimmer source),
        # not a None return. If this KeyErrors, it almost certainly means
        # set_sensors() above failed silently or wasn't applied in time — the
        # channel names themselves are confirmed correct from
        # pyshimmer/dev/channels.py.
        try:
            ax = packet[EChannelType.ACCEL_LN_X]
            ay = packet[EChannelType.ACCEL_LN_Y]
            az = packet[EChannelType.ACCEL_LN_Z]
            gx = packet[EChannelType.GYRO_MPU9150_X]
            gy = packet[EChannelType.GYRO_MPU9150_Y]
            gz = packet[EChannelType.GYRO_MPU9150_Z]
        except KeyError as e:
            self.get_logger().error(
                f"Shimmer packet missing expected channel {e} — ACCEL_LN/GYRO were "
                f"likely not enabled (check the 'Failed to enable ACCEL_LN/GYRO "
                f"sensors' error at startup) or the device is still using a stale "
                f"sensor config from a previous session. Dropping this packet."
            )
            return
        except Exception as e:
            self.get_logger().error(f"Failed to parse Shimmer data packet: {e}")
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # RAW ADC COUNTS, not m/s^2 / rad/s — calibration is not implemented yet
        # (see module docstring). Do NOT apply the deg->rad conversion here; these
        # are not degrees, they're uncalibrated counts, so no unit conversion is
        # meaningful until real calibration is added.
        msg.linear_acceleration.x = float(ax)
        msg.linear_acceleration.y = float(ay)
        msg.linear_acceleration.z = float(az)

        msg.angular_velocity.x = float(gx)
        msg.angular_velocity.y = float(gy)
        msg.angular_velocity.z = float(gz)

        # No orientation estimate produced here — fusion happens downstream.
        msg.orientation_covariance[0] = -1.0
        # Values above are raw/uncalibrated — mark covariance as unknown too, so
        # nothing downstream mistakes these for trustworthy physical quantities.
        msg.linear_acceleration_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = -1.0

        self.pub.publish(msg)

    def destroy_node(self):
        if self.shim_dev is not None:
            try:
                self.shim_dev.stop_streaming()
                self.shim_dev.shutdown()
                self.get_logger().info("Shimmer stream stopped and connection closed")
            except Exception as e:
                self.get_logger().warn(f"Error while shutting down Shimmer connection: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ShimmerImuNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node is not None:
            node.get_logger().error(f"Node crashed: {e}")
        else:
            print(f"Node failed to start: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()