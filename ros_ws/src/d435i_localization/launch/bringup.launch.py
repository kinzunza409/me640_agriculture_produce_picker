import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace to push the whole IMU chain under, if any',
    )

    # Placeholder camera -> base_link mount offset. Replace these defaults
    # with the real measured translation/rotation once known; nothing else
    # in this launch file needs to change.
    camera_x_arg = DeclareLaunchArgument('camera_x', default_value='0.0')
    camera_y_arg = DeclareLaunchArgument('camera_y', default_value='0.0')
    camera_z_arg = DeclareLaunchArgument('camera_z', default_value='0.0')
    camera_roll_arg = DeclareLaunchArgument('camera_roll', default_value='0.0')
    camera_pitch_arg = DeclareLaunchArgument('camera_pitch', default_value='0.0')
    camera_yaw_arg = DeclareLaunchArgument('camera_yaw', default_value='0.0')

    realsense_launch_dir = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch'
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_launch_dir, 'rs_launch.py')
        ),
        launch_arguments={
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',  # 0=none, 1=copy, 2=linear_interpolation (integer param, not the name)
        }.items(),
    )

    # realsense publishes camera_link -> ... -> camera_imu_optical_frame itself
    # (visible in /tf_static); this closes the chain from base_link.
    base_link_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera',
        arguments=[
            '--x', LaunchConfiguration('camera_x'),
            '--y', LaunchConfiguration('camera_y'),
            '--z', LaunchConfiguration('camera_z'),
            '--roll', LaunchConfiguration('camera_roll'),
            '--pitch', LaunchConfiguration('camera_pitch'),
            '--yaw', LaunchConfiguration('camera_yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
    )

    madgwick_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick_node',
        parameters=[{
            'use_mag': False,
            'publish_tf': False,
        }],
        remappings=[
            ('imu/data_raw', 'camera/camera/imu'),
        ],
    )

    ekf_config_path = os.path.join(
        get_package_share_directory('d435i_localization'), 'config', 'ekf.yaml'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config_path],
    )

    imu_chain = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        realsense_launch,
        base_link_to_camera,
        madgwick_node,
        ekf_node,
    ])

    return LaunchDescription([
        namespace_arg,
        camera_x_arg,
        camera_y_arg,
        camera_z_arg,
        camera_roll_arg,
        camera_pitch_arg,
        camera_yaw_arg,
        imu_chain,
    ])
