FROM ros:jazzy-ros-base-noble AS kortex-builder

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

ARG KORTEX_COMMIT=63944dee465d836ea714a114ea3657dfa1617d95
ARG KORTEX_API_URL=https://artifactory.kinovaapps.com/artifactory/generic-public/kortex/API/2.8.0/linux_x86-64_gcc_5.4.zip
ARG KORTEX_API_MD5=b9b68885df8e1dd0e5a2ba7285f14c0d
ARG KORTEX_DRIVER_URL=https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.5.0/linux_x86-64_x86_gcc.zip
ARG KORTEX_DRIVER_MD5=64bd86e7ab8bda90ef1fc7d6a356e080

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    curl \
    git \
    python3-colcon-common-extensions \
    unzip \
    ros-jazzy-hardware-interface \
    ros-jazzy-pluginlib \
    ros-jazzy-rclcpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp/kortex_ws
RUN git clone https://github.com/Kinovarobotics/ros2_kortex.git src/ros2_kortex_official \
    && git -C src/ros2_kortex_official checkout --detach "${KORTEX_COMMIT}" \
    && curl --fail --location --retry 3 --output /tmp/kortex_api.zip "${KORTEX_API_URL}" \
    && echo "${KORTEX_API_MD5}  /tmp/kortex_api.zip" | md5sum --check --status \
    && curl --fail --location --retry 3 --output /tmp/kortex_driver.zip "${KORTEX_DRIVER_URL}" \
    && echo "${KORTEX_DRIVER_MD5}  /tmp/kortex_driver.zip" | md5sum --check --status \
    && mkdir -p vendor/kortex_api_2.8.0 vendor/kortex_driver_2.5.0 \
    && unzip -q /tmp/kortex_api.zip -d vendor/kortex_api_2.8.0 \
    && unzip -q /tmp/kortex_driver.zip -d vendor/kortex_driver_2.5.0

RUN source /opt/ros/jazzy/setup.bash \
    && colcon --log-base log build \
      --base-paths src/ros2_kortex_official \
      --build-base build \
      --install-base /opt/kinova_kortex \
      --packages-select kortex_api \
      --cmake-args -Wno-dev \
        -DFETCHCONTENT_SOURCE_DIR_KINOVA_BINARY_API=/tmp/kortex_ws/vendor/kortex_api_2.8.0 \
    && source /opt/kinova_kortex/setup.bash \
    && colcon --log-base log build \
      --base-paths src/ros2_kortex_official \
      --build-base build \
      --install-base /opt/kinova_kortex \
      --packages-select kortex_driver \
      --cmake-args -Wno-dev \
        -DFETCHCONTENT_SOURCE_DIR_KINOVA_BINARY_API=/tmp/kortex_ws/vendor/kortex_driver_2.5.0

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
    tmux \
    python3-rosdep \
    python3-vcstool \
    unzip \
    ros-jazzy-geometry-msgs \
    ros-jazzy-hardware-interface \
    ros-jazzy-kinova-gen3-7dof-robotiq-2f-85-moveit-config \
    ros-jazzy-moveit-servo \
    ros-jazzy-pluginlib \
    ros-jazzy-rclcpp \
    ros-jazzy-rclpy \
    ros-jazzy-rmw-fastrtps-cpp \
    ros-jazzy-ros2controlcli \
    && rm -rf /var/lib/apt/lists/*

COPY --from=kortex-builder /opt/kinova_kortex /opt/kinova_kortex

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
    && echo 'source /opt/kinova_kortex/setup.bash' >> /root/.bashrc \
    && echo '[ -f /project/ros_ws/install/setup.bash ] && source /project/ros_ws/install/setup.bash' >> /root/.bashrc

WORKDIR /project
