FROM docker.io/library/ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    tmux \
    ros-jazzy-geometry-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-rclpy \
    ros-jazzy-launch \
    ros-jazzy-launch-ros \
    ros-jazzy-robot-localization \
    ros-jazzy-imu-filter-madgwick \
    ros-jazzy-tf2-ros \
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

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
    && echo 'source /opt/realsense_ws/install/setup.bash' >> /root/.bashrc \
    && echo '[ -f /project/ros_ws/install/setup.bash ] && source /project/ros_ws/install/setup.bash' >> /root/.bashrc

WORKDIR /project
