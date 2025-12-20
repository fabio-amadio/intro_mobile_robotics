#!/usr/bin/env bash

CONTAINER_NAME=turtlebot3
IMAGE_NAME=turtlebot3-sim:humble-latest
COLCON_WS=/root/turtlebot3_ws

isRunning=$(docker ps -f name=${CONTAINER_NAME} | grep -c ${CONTAINER_NAME})

if [ "$isRunning" -eq 0 ]; then
    echo "Starting TurtleBot3 container..."

    # Allow Docker to access X server (Gazebo / RViz)
    xhost +local:docker >/dev/null

    # Remove stopped container if it exists
    docker rm ${CONTAINER_NAME} >/dev/null 2>&1

    docker run \
        -it \
        --name ${CONTAINER_NAME} \
        --network host \
        --ipc host \
        --gpus all \
        -e DISPLAY=${DISPLAY} \
        -e QT_X11_NO_MITSHM=1 \
        -e ROS_DOMAIN_ID=30 \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e __NV_PRIME_RENDER_OFFLOAD=1 \
        -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
        -e __VK_LAYER_NV_optimus=NVIDIA_only \
        -e SDL_AUDIODRIVER=dummy \
        -e ALSOFT_DRIVERS=null \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v /dev/shm:/dev/shm \
        -v ./maps:/root/maps \
        -v ./exercise_teleop:${COLCON_WS}/src/exercise_teleop \
        --device=/dev/dri:/dev/dri \
        --entrypoint /bin/bash \
        ${IMAGE_NAME}

else
    echo "TurtleBot3 container already running."
    docker exec -it ${CONTAINER_NAME} /bin/bash
fi

