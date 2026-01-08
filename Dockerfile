# ROS 2 Humble base
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV COLCON_WS=/root/turtlebot3_ws

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    python3-argcomplete \
    python3-pip \
    python3-evdev \
    python3-colcon-common-extensions \
    build-essential \
    git \
    terminator \
    nano \
    x11-xserver-utils \
    libx11-6 \
    libxext6 \
    libxrender1 \
    libxtst6 \
    libxi6 \
    mesa-utils \
    ros-humble-xacro \
    ros-humble-urdf \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3-msgs \
    ros-humble-dynamixel-sdk \
    && rm -rf /var/lib/apt/lists/*

WORKDIR ${COLCON_WS}

# Install pynput via pip
RUN pip3 install --no-cache-dir pynput

# TurtleBot3 repos
RUN mkdir -p ${COLCON_WS}/src && \
    cd ${COLCON_WS}/src && \
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git && \
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git && \
    git clone -b humble https://github.com/fabio-amadio/turtlebot3_simulations.git

# Build workspace
RUN bash -c "source /opt/ros/humble/setup.bash && \
    cd ${COLCON_WS} && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Convenience environment
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source ${COLCON_WS}/install/setup.bash" >> /root/.bashrc && \
    echo "export TURTLEBOT3_MODEL=burger" >> /root/.bashrc

    
# Pre-create Gazebo config directories and do a headless "warm-up" run
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV DISPLAY=

RUN bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && \
              gazebo --version && \
              timeout 20s gzserver --verbose --pause -s libgazebo_ros_init.so -s libgazebo_ros_factory.so || true"

RUN mkdir -p /root/.gazebo/models && \
    git clone --depth 1 https://github.com/osrf/gazebo_models.git /root/.gazebo/models              

WORKDIR ${COLCON_WS}

COPY ./exercise_navigation ${COLCON_WS}/src/exercise_navigation/

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build

CMD ["bash"]
