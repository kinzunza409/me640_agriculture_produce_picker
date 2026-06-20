#!/usr/bin/env python3
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def setup_and_launch(context, *args, **kwargs):
    setup_path = Path(LaunchConfiguration("setup_path").perform(context))

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("clearpath_gz"), "launch", "simulation.launch.py")
        ),
        launch_arguments={
            "setup_path": str(setup_path) + "/",
            "rviz": LaunchConfiguration("rviz").perform(context),
            "world": LaunchConfiguration("world").perform(context),
        }.items(),
    )]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("setup_path", default_value="/root/clearpath/a200_default"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("world", default_value="warehouse"),
        OpaqueFunction(function=setup_and_launch),
    ])