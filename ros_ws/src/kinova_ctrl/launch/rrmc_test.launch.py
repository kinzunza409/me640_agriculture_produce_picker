from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    rrm_node = Node(
        package='kinova_ctrl',
        executable='rrm_controller',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'test_case': True,
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='a200_0000'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('husky_gz'), 'launch', 'default_sim.launch.py',
            ])),
        ),

        rrm_node,

        RegisterEventHandler(
            OnProcessExit(
                target_action=rrm_node,
                on_exit=lambda event, context: (
                    [Shutdown(reason='rrm_controller crashed')]
                    if event.returncode != 0 else []
                ),
            )
        ),
    ])