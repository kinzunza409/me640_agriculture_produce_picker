FROM osrf/ros:jazzy-desktop-full

# Install devcontainer tools
RUN apt-get update && apt-get install -y \
    qtwayland5 \
    && rm -rf /var/lib/apt/lists/*

# Quality of life packages (mesa-utils/x11-apps = glxinfo, glxgears, xeyes for GUI checks)
RUN apt-get update && apt-get install -y \
    tmux \
    tree \
    fzf \
    mesa-utils \
    x11-apps \
    wget \
    gnupg \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Get newest version of fish
RUN apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository -y ppa:fish-shell/release-4 && \
    apt-get update && apt-get install -y fish

# Clearpath Husky/A200 dependencies
RUN wget -qO - https://packages.osrfoundation.org/gazebo.gpg | gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list 
RUN apt-get update && apt-get install -y \
    gz-harmonic \
    ros-jazzy-ros-gz \
    ros-jazzy-clearpath-simulator \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-robot-localization \
    ros-jazzy-imu-filter-madgwick \
    ros-jazzy-imu-tools \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-message-filters \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# RealSense: build librealsense + realsense-ros from source with the RSUSB
# (user-space libusb) backend instead of the stock apt packages.
#
# The stock ros-jazzy-librealsense2 apt package uses the default V4L2
# kernel-driver backend (/dev/videoN + uvcvideo + hid-sensor-hub). On the
# D435i this reliably triggers a USB re-enumeration when the IMU/HID sensor
# hub is powered on, breaking gyro/accel streaming. FORCE_RSUSB_BACKEND=ON
# builds a user-space libusb backend that bypasses V4L2/uvcvideo/hid entirely.
# This is a build-time-only CMake flag (no runtime/env switch exists), so both
# librealsense and the realsense-ros wrapper must be built from source.
# ---------------------------------------------------------------------------

# librealsense from source (RSUSB backend). Kept in its own layer so it caches
# independently and compile failures are easy to isolate.
ARG LIBREALSENSE_VERSION=v2.58.2
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    libssl-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    && rm -rf /var/lib/apt/lists/* \
    && git clone --depth 1 --branch ${LIBREALSENSE_VERSION} \
        https://github.com/IntelRealSense/librealsense.git /tmp/librealsense \
    && cmake -S /tmp/librealsense -B /tmp/librealsense/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DFORCE_RSUSB_BACKEND=ON \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_GRAPHICAL_EXAMPLES=OFF \
        -DBUILD_PYTHON_BINDINGS=OFF \
        -DBUILD_UNIT_TESTS=OFF \
        -DBUILD_TOOLS=OFF \
        -DBUILD_WITH_OPENMP=ON \
    && cmake --build /tmp/librealsense/build --target install -- -j$(nproc) \
    && ldconfig \
    && rm -rf /tmp/librealsense

# realsense-ros (realsense2_camera + msgs + description) built against the
# custom librealsense above, into an overlay workspace at /opt/realsense_ws.
ARG REALSENSE_ROS_VERSION=4.58.2
RUN mkdir -p /opt/realsense_ws/src \
    && git clone --depth 1 --branch ${REALSENSE_ROS_VERSION} \
        https://github.com/IntelRealSense/realsense-ros.git \
        /opt/realsense_ws/src/realsense-ros \
    && apt-get update \
    && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths /opt/realsense_ws/src --ignore-src -r -y \
        --skip-keys=librealsense2 \
    && rm -rf /var/lib/apt/lists/* \
    && cd /opt/realsense_ws \
    && . /opt/ros/jazzy/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf /opt/realsense_ws/build /opt/realsense_ws/log

# ROS2 settings
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc
RUN echo 'source /opt/realsense_ws/install/setup.bash' >> /root/.bashrc
RUN echo 'source /project/ros_ws/install/setup.bash' >> /root/.bashrc

# Build robot configs for simulation
ARG CLEARPATH_CONFIG_DIR=/tmp/clearpath_config
COPY .devcontainer/build_clearpath_configs.sh /tmp/build_clearpath_configs.sh
COPY ros_ws/config/clearpath $CLEARPATH_CONFIG_DIR
RUN bash /tmp/build_clearpath_configs.sh $CLEARPATH_CONFIG_DIR
# Misc settings for container

COPY .devcontainer/setup.fish /tmp/setup.fish
RUN fish /tmp/setup.fish