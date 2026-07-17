from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('kinova_pose_goal'))

    tracking = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_share / 'launch' / 'servo_tracking.launch.py')
        ),
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'use_internal_bus_gripper_comm': LaunchConfiguration(
                'use_internal_bus_gripper_comm'
            ),
        }.items(),
    )

    fake_target = Node(
        package='kinova_pose_goal',
        executable='fake_target_frame_node',
        name='fake_target_frame',
        output='screen',
        parameters=[str(package_share / 'config' / 'fake_target_frame.yaml')],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('robot_ip', default_value='192.168.1.10'),
            DeclareLaunchArgument(
                'use_fake_hardware',
                default_value='true',
                choices=['true', 'false'],
            ),
            DeclareLaunchArgument(
                'use_internal_bus_gripper_comm',
                default_value='true',
                choices=['true', 'false'],
            ),
            tracking,
            fake_target,
        ]
    )
