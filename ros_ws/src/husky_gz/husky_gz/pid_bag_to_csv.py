#!/usr/bin/env python3
import argparse
import csv
import json
import math
from pathlib import Path

from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


CHASSIS_POSE_TOPIC = '/pid_performance/chassis_pose'
EE_POSE_TOPIC = '/pid_performance/ee_pose'
DYNAMIC_JOINT_STATES_TOPIC = '/a200_0000/dynamic_joint_states'
CONTROLLER_STATE_TOPICS = (
    '/a200_0000/arm_0_joint_trajectory_controller/state',
    '/a200_0000/arm_0_joint_trajectory_controller/controller_state',
)
CMD_VEL_TOPICS = (
    '/a200_0000/cmd_vel',
    '/a200_0000/platform/cmd_vel_unstamped',
)


class CsvSink:
    def __init__(self, path, fieldnames):
        self.path = path
        self.fieldnames = fieldnames
        self.file = None
        self.writer = None
        self.rows = 0

    def write(self, row):
        if self.file is None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.file = self.path.open('w', newline='')
            self.writer = csv.DictWriter(self.file, fieldnames=self.fieldnames)
            self.writer.writeheader()
        self.writer.writerow(row)
        self.rows += 1

    def close(self):
        if self.file is not None:
            self.file.close()


def main():
    parser = argparse.ArgumentParser(
        description='Convert a PID performance ROS bag into CSV files.'
    )
    parser.add_argument('bag_path', help='Path to a rosbag directory')
    parser.add_argument(
        '--output-dir',
        help='CSV output directory. Defaults to <bag_path>_csv.',
    )
    parser.add_argument(
        '--storage-id',
        default=None,
        help='rosbag storage id. Defaults to the bag metadata value or sqlite3.',
    )
    args = parser.parse_args()

    bag_path = Path(args.bag_path).expanduser().resolve()
    output_dir = _resolve_output_dir(bag_path, args.output_dir)
    storage_id = args.storage_id or _infer_storage_id(bag_path)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )

    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    message_types = {name: get_message(type_name) for name, type_name in topic_types.items()}
    topic_counts = {name: 0 for name in topic_types}
    sinks = _create_sinks(output_dir)
    imu_topics = {
        topic for topic, type_name in topic_types.items()
        if type_name == 'sensor_msgs/msg/Imu'
    }

    while reader.has_next():
        topic, data, bag_time_ns = reader.read_next()
        topic_counts[topic] = topic_counts.get(topic, 0) + 1
        msg_type = message_types.get(topic)
        if msg_type is None:
            continue
        msg = deserialize_message(data, msg_type)
        fallback_time = bag_time_ns / 1e9

        if topic == CHASSIS_POSE_TOPIC:
            sinks['chassis_pose'].write(_pose_row(msg, fallback_time))
        elif topic == EE_POSE_TOPIC:
            sinks['ee_pose'].write(_pose_row(msg, fallback_time))
        elif topic == DYNAMIC_JOINT_STATES_TOPIC:
            for row in _dynamic_joint_rows(msg, fallback_time):
                sinks['joint_efforts'].write(row)
        elif topic in CONTROLLER_STATE_TOPICS:
            for row in _controller_rows(msg, fallback_time, topic):
                sinks['controller_tracking'].write(row)
        elif topic in CMD_VEL_TOPICS:
            sinks['cmd_vel'].write(_cmd_vel_row(msg, fallback_time, topic))
        elif topic in imu_topics:
            sinks['imu'].write(_imu_row(msg, fallback_time, topic))

    for sink in sinks.values():
        sink.close()

    metadata = {
        'bag_path': str(bag_path),
        'output_dir': str(output_dir),
        'storage_id': storage_id,
        'pose_source': (
            'Gazebo dynamic pose via /pid_performance/gazebo_dynamic_pose; '
            'odom is recorded as reference only'
        ),
        'topic_types': topic_types,
        'topic_counts': topic_counts,
        'generated_csv': {
            name: str(sink.path) for name, sink in sinks.items() if sink.rows > 0
        },
        'row_counts': {name: sink.rows for name, sink in sinks.items()},
        'missing_optional_topics': _missing_optional_topics(topic_types, imu_topics),
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / 'metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')

    print('Wrote PID performance CSVs to %s' % output_dir)
    for name, sink in sinks.items():
        if sink.rows > 0:
            print('  %s: %s rows' % (sink.path.name, sink.rows))
    print('  metadata.json')


def _resolve_output_dir(bag_path, raw_output_dir):
    if raw_output_dir:
        return Path(raw_output_dir).expanduser().resolve()
    return Path(str(bag_path) + '_csv')


def _create_sinks(output_dir):
    pose_fields = [
        'time_sec', 'frame_id', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
        'roll', 'pitch', 'yaw',
    ]
    return {
        'chassis_pose': CsvSink(output_dir / 'chassis_pose.csv', pose_fields),
        'ee_pose': CsvSink(output_dir / 'ee_pose.csv', pose_fields),
        'joint_efforts': CsvSink(
            output_dir / 'joint_efforts.csv',
            ['time_sec', 'joint_name', 'position', 'velocity', 'effort'],
        ),
        'controller_tracking': CsvSink(
            output_dir / 'controller_tracking.csv',
            [
                'time_sec', 'topic', 'joint_name',
                'reference_position', 'feedback_position', 'error_position',
                'reference_velocity', 'feedback_velocity', 'error_velocity',
            ],
        ),
        'cmd_vel': CsvSink(
            output_dir / 'cmd_vel.csv',
            [
                'time_sec', 'topic', 'linear_x', 'linear_y', 'linear_z',
                'angular_x', 'angular_y', 'angular_z',
            ],
        ),
        'imu': CsvSink(
            output_dir / 'imu.csv',
            [
                'time_sec', 'topic', 'frame_id', 'qx', 'qy', 'qz', 'qw',
                'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
                'angular_velocity_norm', 'linear_acceleration_x',
                'linear_acceleration_y', 'linear_acceleration_z',
                'linear_acceleration_norm',
            ],
        ),
    }


def _infer_storage_id(bag_path):
    metadata_path = bag_path / 'metadata.yaml'
    if metadata_path.exists():
        for line in metadata_path.read_text(errors='ignore').splitlines():
            if 'storage_identifier:' in line:
                value = line.split(':', 1)[1].strip()
                if value:
                    return value
    return 'sqlite3'


def _time_sec(msg, fallback_time):
    header = getattr(msg, 'header', None)
    stamp = getattr(header, 'stamp', None)
    if stamp is None:
        return fallback_time
    sec = float(getattr(stamp, 'sec', 0))
    nanosec = float(getattr(stamp, 'nanosec', 0))
    if sec == 0.0 and nanosec == 0.0:
        return fallback_time
    return sec + nanosec / 1e9


def _pose_row(msg, fallback_time):
    q = msg.pose.orientation
    roll, pitch, yaw = _quat_to_euler(q.x, q.y, q.z, q.w)
    return {
        'time_sec': _time_sec(msg, fallback_time),
        'frame_id': msg.header.frame_id,
        'x': msg.pose.position.x,
        'y': msg.pose.position.y,
        'z': msg.pose.position.z,
        'qx': q.x,
        'qy': q.y,
        'qz': q.z,
        'qw': q.w,
        'roll': roll,
        'pitch': pitch,
        'yaw': yaw,
    }


def _dynamic_joint_rows(msg, fallback_time):
    time_sec = _time_sec(msg, fallback_time)
    for joint_name, interface_value in zip(msg.joint_names, msg.interface_values):
        values = dict(zip(interface_value.interface_names, interface_value.values))
        if 'effort' not in values:
            continue
        yield {
            'time_sec': time_sec,
            'joint_name': joint_name,
            'position': values.get('position', ''),
            'velocity': values.get('velocity', ''),
            'effort': values.get('effort', ''),
        }


def _controller_rows(msg, fallback_time, topic):
    time_sec = _time_sec(msg, fallback_time)
    reference = _point_with_fallback(msg, 'reference', 'desired')
    feedback = _point_with_fallback(msg, 'feedback', 'actual')
    error = getattr(msg, 'error')
    for index, joint_name in enumerate(msg.joint_names):
        yield {
            'time_sec': time_sec,
            'topic': topic,
            'joint_name': joint_name,
            'reference_position': _value_at(reference.positions, index),
            'feedback_position': _value_at(feedback.positions, index),
            'error_position': _value_at(error.positions, index),
            'reference_velocity': _value_at(reference.velocities, index),
            'feedback_velocity': _value_at(feedback.velocities, index),
            'error_velocity': _value_at(error.velocities, index),
        }


def _point_with_fallback(msg, preferred, fallback):
    point = getattr(msg, preferred)
    if _point_has_values(point):
        return point
    return getattr(msg, fallback)


def _point_has_values(point):
    return bool(point.positions or point.velocities or point.accelerations or point.effort)


def _value_at(values, index):
    return values[index] if index < len(values) else ''


def _cmd_vel_row(msg, fallback_time, topic):
    return {
        'time_sec': fallback_time,
        'topic': topic,
        'linear_x': msg.linear.x,
        'linear_y': msg.linear.y,
        'linear_z': msg.linear.z,
        'angular_x': msg.angular.x,
        'angular_y': msg.angular.y,
        'angular_z': msg.angular.z,
    }


def _imu_row(msg, fallback_time, topic):
    av = msg.angular_velocity
    la = msg.linear_acceleration
    q = msg.orientation
    return {
        'time_sec': _time_sec(msg, fallback_time),
        'topic': topic,
        'frame_id': msg.header.frame_id,
        'qx': q.x,
        'qy': q.y,
        'qz': q.z,
        'qw': q.w,
        'angular_velocity_x': av.x,
        'angular_velocity_y': av.y,
        'angular_velocity_z': av.z,
        'angular_velocity_norm': math.sqrt(av.x * av.x + av.y * av.y + av.z * av.z),
        'linear_acceleration_x': la.x,
        'linear_acceleration_y': la.y,
        'linear_acceleration_z': la.z,
        'linear_acceleration_norm': math.sqrt(la.x * la.x + la.y * la.y + la.z * la.z),
    }


def _missing_optional_topics(topic_types, imu_topics):
    missing = []
    if not imu_topics:
        missing.append('sensor_msgs/msg/Imu topic')
    for topic in (CHASSIS_POSE_TOPIC, EE_POSE_TOPIC, DYNAMIC_JOINT_STATES_TOPIC):
        if topic not in topic_types:
            missing.append(topic)
    if not any(topic in topic_types for topic in CONTROLLER_STATE_TOPICS):
        missing.append('arm joint trajectory controller state topic')
    return missing


def _quat_to_euler(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        x, y, z, w = 0.0, 0.0, 0.0, 1.0
    else:
        x, y, z, w = x / norm, y / norm, z / norm, w / norm

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


if __name__ == '__main__':
    main()
