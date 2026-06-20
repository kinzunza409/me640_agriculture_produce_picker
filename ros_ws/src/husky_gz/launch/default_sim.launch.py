#!/usr/bin/env python3
import os
import shutil
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup_and_launch(context, *args, **kwargs):
    # Resolve launch args into real strings at runtime
    setup_path = Path(LaunchConfiguration("setup_path").perform(context))
    robot_config = LaunchConfiguration("robot_config").perform(context)

    # Ensure the setup directory exists then drop the config in as robot.yaml
    # (clearpath_gz expects to find it at <setup_path>/robot.yaml)
    setup_path.mkdir(exist_ok=True)
    shutil.copy(robot_config, setup_path / "robot.yaml")

    # Hand off to the clearpath_gz sim launch with rviz enabled
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("clearpath_gz"), "launch", "simulation.launch.py")
        ),
        launch_arguments={
            "setup_path": str(setup_path) + "/",
            "rviz": LaunchConfiguration("rviz").perform(context),
        }.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        # OpaqueFunction defers setup_and_launch to runtime so we can
        # resolve args and do file operations before the sim starts
        DeclareLaunchArgument("setup_path", default_value="/tmp/clearpath"),
        DeclareLaunchArgument("robot_config", default_value="/project/ros_ws/config/husky/a200_default.yaml"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=setup_and_launch),
    ])