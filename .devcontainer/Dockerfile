FROM osrf/ros:humble-desktop-full

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

# Clearpath Husky/A200 simulation dependencies
RUN wget -qO - https://packages.osrfoundation.org/gazebo.gpg | gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list \
RUN apt-get update && apt-get install -y \
    ignition-fortress \
    ros-humble-clearpath-simulator \
    ros-humble-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# ROS2 settings
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc

# Misc settings for container
COPY setup.fish /tmp/setup.fish
RUN fish /tmp/setup.fish