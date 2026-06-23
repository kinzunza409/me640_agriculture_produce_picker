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

    # ── Controller ──────────────────────────────────────────────────
    pid_node = Node(
        package='kinova_ctrl',
        executable='pid_controller',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        # stock TransformListener subs to absolute /tf(_static); remap to
        # relative so the node namespace resolves them to /<ns>/tf(_static)
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='a200_0000'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # ── Simulation ──────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('husky_gz'), 'launch', 'default_sim.launch.py',
            ])),
        ),

        # ── Controller ──────────────────────────────────────────────
        pid_node,

        # Tear down the whole launch (incl. sim) if pid_controller crashes
        # (nonzero exit); a clean exit leaves the sim running.
        RegisterEventHandler(
            OnProcessExit(
                target_action=pid_node,
                on_exit=lambda event, context: (
                    [Shutdown(reason='pid_controller crashed')]
                    if event.returncode != 0 else []
                ),
            )
        ),

        # ── Target publisher ────────────────────────────────────────
        Node(
            package='kinova_ctrl',
            executable='point_trajectory',
            namespace=namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])