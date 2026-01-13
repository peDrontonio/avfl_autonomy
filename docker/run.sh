#!/bin/bash

IMAGE_NAME="ardupilot-gazebo-ros2"
IMAGE_TAG="latest"
CONTAINER_NAME="ardupilot-sim"

# Get the workspace directory (parent of docker folder)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "${SCRIPT_DIR}")"

# Allow X server connections from Docker
xhost +local:docker 2>/dev/null

echo "=========================================="
echo "Starting container: ${CONTAINER_NAME}"
echo "Image: ${IMAGE_NAME}:${IMAGE_TAG}"
echo "Mounting workspace: ${WORKSPACE_DIR}"
echo "=========================================="

docker run -it --rm \
    --name ${CONTAINER_NAME} \
    --net=host \
    --privileged \
    -e DISPLAY=${DISPLAY} \
    -e QT_X11_NO_MITSHM=1 \
    -e LIBGL_ALWAYS_SOFTWARE=0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev/dri:/dev/dri \
    -v /dev/input:/dev/input \
    -v "${WORKSPACE_DIR}:/root/internship_ws/src/avfl_autonomy:rw" \
    -v "${HOME}/.gazebo:/root/.gazebo:rw" \
    ${IMAGE_NAME}:${IMAGE_TAG} \
    "$@"

# Revoke X server access when done
xhost -local:docker 2>/dev/null
