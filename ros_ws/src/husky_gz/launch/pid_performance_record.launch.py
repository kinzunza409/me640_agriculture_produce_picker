#!/usr/bin/env python3
from datetime import datetime
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


FORWARDED_SIM_ARGUMENTS = ('setup_path', 'rviz', 'world', 'x', 'y', 'z', 'yaw')

DEFAULT_RECORD_REGEX = (
    r'^(/clock'
    r'|/pid_performance/(chassis_pose|ee_pose|gazebo_dynamic_pose)'
    r'|/a200_0000/(tf|tf_static)'
    r'|/a200_0000/platform/(odom|odom/filtered|cmd_vel_unstamped)'
    r'|/a200_0000/(cmd_vel|dynamic_joint_states)'
    r'|/a200_0000/arm_0_joint_trajectory_controller/(state|controller_state)'
    r'|.*imu.*'
    r'|.*pid.*)$'
)


def _resolve_bag_path(raw_path):
    bag_path = Path(raw_path).expanduser()
    if bag_path.exists():
        suffix = datetime.now().strftime('%Y%m%d_%H%M%S')
        bag_path = bag_path.with_name(f'{bag_path.name}_{suffix}')
    bag_path.parent.mkdir(parents=True, exist_ok=True)
    return bag_path


def _launch_setup(context, *args, **kwargs):
    package_share = get_package_share_directory('husky_gz')
    default_sim_launch = os.path.join(package_share, 'launch', 'default_sim.launch.py')

    sim_arguments = {
        name: LaunchConfiguration(name).perform(context)
        for name in FORWARDED_SIM_ARGUMENTS
    }

    world = LaunchConfiguration('world').perform(context)
    dynamic_pose_gz_topic = f'/world/{world}/dynamic_pose/info'
    bag_path = _resolve_bag_path(LaunchConfiguration('bag_path').perform(context))
    record_delay = float(LaunchConfiguration('record_delay').perform(context))
    pose_rate = float(LaunchConfiguration('pose_rate').perform(context))

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(default_sim_launch),
            launch_arguments=sim_arguments.items(),
            condition=IfCondition(LaunchConfiguration('launch_sim')),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='pid_gazebo_pose_bridge',
            output='screen',
            arguments=[
                f'{dynamic_pose_gz_topic}@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            ],
            remappings=[
                (dynamic_pose_gz_topic, LaunchConfiguration('dynamic_pose_topic')),
            ],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
        Node(
            package='husky_gz',
            executable='pid_pose_logger',
            name='pid_pose_logger',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'world_frame': LaunchConfiguration('world_frame'),
                'chassis_frame': LaunchConfiguration('chassis_frame'),
                'ee_frame': LaunchConfiguration('ee_frame'),
                'publish_rate': pose_rate,
                'dynamic_pose_topic': LaunchConfiguration('dynamic_pose_topic'),
            }],
            remappings=[
                ('/tf', LaunchConfiguration('tf_topic')),
                ('/tf_static', LaunchConfiguration('tf_static_topic')),
            ],
        ),
        TimerAction(
            period=record_delay,
            actions=[
                LogInfo(msg=f'Recording PID performance bag to: {bag_path}'),
                ExecuteProcess(
                    cmd=[
                        'ros2',
                        'bag',
                        'record',
                        '-o',
                        str(bag_path),
                        '--regex',
                        LaunchConfiguration('record_regex'),
                    ],
                    output='screen',
                ),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('launch_sim', default_value='true'),
        DeclareLaunchArgument('bag_path', default_value='/project/ros_ws/bags/pid_performance'),
        DeclareLaunchArgument('record_delay', default_value='10.0'),
        DeclareLaunchArgument('record_regex', default_value=DEFAULT_RECORD_REGEX),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('pose_rate', default_value='20.0'),
        DeclareLaunchArgument('world_frame', default_value='world'),
        DeclareLaunchArgument('chassis_frame', default_value='base_link'),
        DeclareLaunchArgument('ee_frame', default_value='arm_0_end_effector_link'),
        DeclareLaunchArgument(
            'dynamic_pose_topic',
            default_value='/pid_performance/gazebo_dynamic_pose',
        ),
        DeclareLaunchArgument('tf_topic', default_value='/a200_0000/tf'),
        DeclareLaunchArgument('tf_static_topic', default_value='/a200_0000/tf_static'),
        DeclareLaunchArgument('setup_path', default_value='/root/clearpath/a200_gen3_default'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('world', default_value='rough_terrain'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.3'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=_launch_setup),
    ])
