#!/usr/bin/env python3
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

FORWARDED_ARGUMENTS = ("rviz", "world", "x", "y", "z", "yaw")


def clearpath_launch_arguments(context):
    setup_path = Path(LaunchConfiguration("setup_path").perform(context))
    arguments = {"setup_path": str(setup_path) + "/"}
    arguments.update({
        name: LaunchConfiguration(name).perform(context)
        for name in FORWARDED_ARGUMENTS
    })
    return arguments


def setup_and_launch(context, *args, **kwargs):
    clearpath_sim_launch = os.path.join(
        get_package_share_directory("clearpath_gz"),
        "launch",
        "simulation.launch.py",
    )

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(clearpath_sim_launch),
        launch_arguments=clearpath_launch_arguments(context).items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("setup_path", default_value="/root/clearpath/a200_gen3_default"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("world", default_value="rough_terrain"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.3"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        OpaqueFunction(function=setup_and_launch),
    ])
