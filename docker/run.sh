#!/bin/bash

IMAGE_NAME="ardupilot-gazebo-ros2"
IMAGE_TAG="humble-harmonic"
CONTAINER_NAME="ardupilot-sim"

# Get the workspace directory (parent of docker folder)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "${SCRIPT_DIR}")"

# Check if we want to exec into existing container
if [ "$1" == "exec" ] || [ "$1" == "attach" ]; then
    # Try to find the running container
    CONTAINER_ID=$(docker ps --filter "name=${CONTAINER_NAME}" --format "{{.ID}}" | head -n 1)
    
    if [ -z "$CONTAINER_ID" ]; then
        echo "=========================================="
        echo "Error: No running container found!"
        echo "Make sure the container is running first."
        echo "=========================================="
        exit 1
    fi
    
    echo "=========================================="
    echo "Opening new terminal in container: ${CONTAINER_NAME}"
    echo "=========================================="
    
    docker exec -it ${CONTAINER_ID} /bin/bash
    exit 0
fi

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
