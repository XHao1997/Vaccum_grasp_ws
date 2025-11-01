# Ubuntu 20.04 + ROS Noetic desktop-full (RViz, Gazebo Classic, rqt)
FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"]

# Basics + GUI/OpenGL bits for RViz/Gazebo + catkin tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo git curl wget nano less build-essential cmake \
    python3-pip python3-catkin-tools python3-rosdep \
    mesa-utils libgl1-mesa-dri libgl1-mesa-glx \
    x11-apps \
    ros-noetic-moveit* \
    ros-noetic-gazebo-ros* \
    ros-noetic-effort-controllers \
    ros-noetic-joint-trajectory-controller \
    ros-noetic-rqt-joint-trajectory-controller \
    && rm -rf /var/lib/apt/lists/*

# Create non-root user that matches your host UID/GID (override via build args)
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && usermod -aG sudo ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Initialize rosdep (as root), then hand control to user
RUN rosdep init || true
USER ${USERNAME}
RUN rosdep update

# ROS env + quality-of-life
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/${USERNAME}/.bashrc
ENV QT_X11_NO_MITSHM=1
WORKDIR /home/${USERNAME}/catkin_ws
