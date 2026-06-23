from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
    TimerAction,          # Used to delay rrm_node startup
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---------------------------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------------------------
    # 'namespace' scopes all nodes so multiple robots can coexist without topic
    # collisions (e.g. a200_0000, a200_0001, …).
    namespace = LaunchConfiguration('namespace')

    # 'use_sim_time' makes nodes consume /clock from Gazebo instead of wall time.
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ---------------------------------------------------------------------------
    # Node: base_link_world_aligned  (husky_gz)
    # ---------------------------------------------------------------------------
    # Publishes the 'base_link_world_aligned' TF frame that rrm_controller
    # depends on as its world_frame.  Must be alive before the controller starts.
    base_link_world_aligned_node = Node(
        package='husky_gz',
        executable='base_link_world_aligned',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        # Remap global /tf and /tf_static into the namespace so the TF trees
        # stay isolated per robot.
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    # ---------------------------------------------------------------------------
    # Node: rrm_controller  (kinova_ctrl)
    # ---------------------------------------------------------------------------
    # Reactive/receding-horizon motion controller for the Kinova arm.
    # Launched via TimerAction below so the world frame is guaranteed to exist.
    rrm_node = Node(
        package='kinova_ctrl',
        executable='rrm_controller',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'test_case': True,
            # Must match the frame name published by base_link_world_aligned.
            'world_frame': 'base_link_world_aligned',
            'test_target_pose': [0.2, 0.8, 0.8, -0.7071, 0.0, 0.0, 0.7071]
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    # ---------------------------------------------------------------------------
    # Delayed launch of rrm_node
    # ---------------------------------------------------------------------------
    # Wait 2 s after base_link_world_aligned starts before bringing up the
    # controller, giving the frame time to be published on the TF tree.
    # Adjust the delay if your hardware/sim takes longer to initialise.
    delayed_rrm_node = RegisterEventHandler(
        OnProcessStart(
            target_action=base_link_world_aligned_node,
            on_start=[
                TimerAction(
                    period=2.0,   # seconds — tune as needed
                    actions=[rrm_node],
                )
            ],
        )
    )

    # ---------------------------------------------------------------------------
    # Crash guard: shut the whole launch down if rrm_controller exits with an
    # error.  A clean exit (returncode == 0) is ignored so intentional stops
    # (e.g. end of test) don't kill the simulator.
    # ---------------------------------------------------------------------------
    rrm_crash_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=rrm_node,
            on_exit=lambda event, context: (
                [Shutdown(reason='rrm_controller crashed')]
                if event.returncode != 0 else []
            ),
        )
    )

    # ---------------------------------------------------------------------------
    # Launch description assembly
    # ---------------------------------------------------------------------------
    return LaunchDescription([
        # Declare configurable arguments with sensible defaults.
        DeclareLaunchArgument('namespace', default_value='a200_0000'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # Start Gazebo simulation (loads world, robot description, etc.).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('husky_gz'), 'launch', 'default_sim.launch.py',
            ])),
        ),

        # Bring up the world-frame publisher immediately.
        base_link_world_aligned_node,

        # Start rrm_controller 2 s after base_link_world_aligned is running.
        delayed_rrm_node,

        # Propagate rrm_controller crashes to the whole launch process.
        rrm_crash_handler,
    ])