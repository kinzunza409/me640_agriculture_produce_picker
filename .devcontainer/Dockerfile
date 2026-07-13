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
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Get newest version of fish
RUN apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository -y ppa:fish-shell/release-4 && \
    apt-get update && apt-get install -y fish

# Clearpath Husky/A200 simulation dependencies
RUN wget -qO - https://packages.osrfoundation.org/gazebo.gpg | gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list 
RUN apt-get update && apt-get install -y \
    gz-harmonic \
    ros-jazzy-ros-gz \
    ros-jazzy-clearpath-simulator \
    ros-jazzy-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# Python3 dependencies
RUN pip install pyshimmer --break-system-packages

# ROS2 settings
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc
RUN echo 'source /project/ros_ws/install/setup.bash' >> /root/.bashrc

# Build robot configs for simulation
ARG CLEARPATH_CONFIG_DIR=/tmp/clearpath_config
COPY .devcontainer/build_clearpath_configs.sh /tmp/build_clearpath_configs.sh
COPY ros_ws/config/clearpath $CLEARPATH_CONFIG_DIR
RUN bash /tmp/build_clearpath_configs.sh $CLEARPATH_CONFIG_DIR

# Misc settings for container
COPY .devcontainer/setup.fish /tmp/setup.fish
RUN fish /tmp/setup.fish