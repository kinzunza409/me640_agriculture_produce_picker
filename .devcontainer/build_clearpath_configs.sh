#!/bin/bash
# Generates all Clearpath robot configuration files from robot.yaml configs.
# Run this once (or on config changes) before launching the simulation.
# Output lands in ~/clearpath/<config_name>/ for each yaml found.

source /opt/ros/jazzy/setup.bash

CONFIG_DIR="${1:-/tmp/clearpath_config}"

find "$CONFIG_DIR" -name "*.yaml" | while read -r yaml_file; do
    # Use the yaml filename (without extension) as a unique output directory
    # e.g. a200_kinova.yaml -> ~/clearpath/a200_kinova/
    name=$(basename "$yaml_file" .yaml)
    setup_path="$HOME/clearpath/$name"

    mkdir -p "$setup_path"
    cp "$yaml_file" "$setup_path/robot.yaml"  # Generators expect robot.yaml at the setup_path root

    # Generate environment variables and workspace sourcing for robot upstart
    ros2 run clearpath_generator_common generate_bash -s "$setup_path/"

    # Generate robot.urdf.xacro from the platform/mounts/sensors/manipulators config
    ros2 run clearpath_generator_common generate_description -s "$setup_path/"

    # Generate robot.srdf for MoveIt — disables out-of-range link collisions for faster planning
    ros2 run clearpath_generator_common generate_semantic_description -s "$setup_path/"

    # Generate Gazebo simulation launch files
    ros2 run clearpath_generator_gz generate_launch -s "$setup_path/"

    # Generate ROS 2 parameter files for all nodes (control, MoveIt, sensors, etc.)
    ros2 run clearpath_generator_gz generate_param -s "$setup_path/"
done