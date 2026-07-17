FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    curl \
    git \
    iproute2 \
    iputils-ping \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    unzip \
    ros-jazzy-geometry-msgs \
    ros-jazzy-kinova-gen3-7dof-robotiq-2f-85-moveit-config \
    ros-jazzy-moveit-servo \
    ros-jazzy-rclcpp \
    ros-jazzy-rclpy \
    ros-jazzy-rmw-fastrtps-cpp \
    ros-jazzy-ros2controlcli \
    && rm -rf /var/lib/apt/lists/*

# Jazzy's released Kortex description and Robotiq description use different
# xacro argument names. Keep the compatibility change explicit and reviewable.
RUN apt-get update && apt-get install -y --no-install-recommends patch \
    && rm -rf /var/lib/apt/lists/*
COPY patches/ros2_kortex-jazzy-robotiq-xacro.patch /tmp/ros2_kortex-jazzy-robotiq-xacro.patch
RUN cd /opt/ros/jazzy/share \
    && sed -i 's/\r$//' kortex_description/grippers/robotiq_2f_85/urdf/robotiq_2f_85_macro.xacro \
    && patch --batch --forward -p1 < /tmp/ros2_kortex-jazzy-robotiq-xacro.patch \
    && rm /tmp/ros2_kortex-jazzy-robotiq-xacro.patch

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
    && echo '[ -f /project/ros_ws/install_jazzy/setup.bash ] && source /project/ros_ws/install_jazzy/setup.bash' >> /root/.bashrc \
    && echo '[ -f /project/ros_ws/install_target_tracking/setup.bash ] && source /project/ros_ws/install_target_tracking/setup.bash' >> /root/.bashrc

WORKDIR /project
