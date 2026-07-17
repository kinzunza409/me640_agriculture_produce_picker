from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _launch_setup(context):
    robot_ip = LaunchConfiguration('robot_ip').perform(context)
    use_fake_hardware = LaunchConfiguration('use_fake_hardware').perform(context)
    use_internal_bus = LaunchConfiguration(
        'use_internal_bus_gripper_comm'
    ).perform(context)

    mappings = {
        'robot_ip': robot_ip,
        'use_fake_hardware': use_fake_hardware,
        'gripper': 'robotiq_2f_85',
        'gripper_joint_name': 'robotiq_85_left_knuckle_joint',
        'dof': '7',
        'gripper_max_velocity': '100.0',
        'gripper_max_force': '100.0',
        'use_internal_bus_gripper_comm': use_internal_bus,
    }

    moveit_config = (
        MoveItConfigsBuilder(
            'gen3',
            package_name='kinova_gen3_7dof_robotiq_2f_85_moveit_config',
        )
        .robot_description(mappings=mappings)
        .to_moveit_configs()
    )

    package_share = Path(get_package_share_directory('kinova_pose_goal'))
    with (package_share / 'config' / 'servo_tracking.yaml').open() as stream:
        servo_parameters = yaml.safe_load(stream)

    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        name='servo_node',
        output='screen',
        parameters=[
            {'moveit_servo': servo_parameters},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    tracker_node = Node(
        package='kinova_pose_goal',
        executable='pose_tracking_node',
        name='kinova_pose_tracker',
        output='screen',
        parameters=[str(package_share / 'config' / 'pose_tracking.yaml')],
    )

    select_pose_commands = TimerAction(
        period=1.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'service',
                    'call',
                    '/servo_node/switch_command_type',
                    'moveit_msgs/srv/ServoCommandType',
                    '{command_type: 2}',
                ],
                output='screen',
            )
        ],
    )

    return [servo_node, tracker_node, select_pose_commands]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_ip',
                default_value='192.168.1.10',
                description='Kinova Gen3 IPv4 address.',
            ),
            DeclareLaunchArgument(
                'use_fake_hardware',
                default_value='false',
                choices=['true', 'false'],
            ),
            DeclareLaunchArgument(
                'use_internal_bus_gripper_comm',
                default_value='true',
                choices=['true', 'false'],
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
