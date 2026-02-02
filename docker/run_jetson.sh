#!/bin/bash
# ============================================
# AVFL AUTONOMY - JETSON RUN SCRIPT
# Runs Docker container for real drone operations
# ============================================

IMAGE_NAME="avfl-jetson"
IMAGE_TAG="humble"
CONTAINER_NAME="avfl-drone"

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

# DDS Agent mode - runs MicroXRCEAgent for ArduPilot communication
if [ "$1" == "dds" ]; then
    SERIAL_PORT="${2:-/dev/ttyACM0}"
    BAUD_RATE="${3:-115200}"
    
    echo "=========================================="
    echo "Starting DDS Agent on ${SERIAL_PORT} @ ${BAUD_RATE}"
    echo "=========================================="
    
    docker run -it --rm \
        --name ${CONTAINER_NAME}-dds \
        --net=host \
        --privileged \
        -v /dev:/dev \
        ${IMAGE_NAME}:${IMAGE_TAG} \
        MicroXRCEAgent serial --dev ${SERIAL_PORT} -b ${BAUD_RATE}
    exit 0
fi

# UDP Agent mode - runs micro_ros_agent for UDP communication
if [ "$1" == "udp" ]; then
    UDP_PORT="${2:-2019}"
    
    echo "=========================================="
    echo "Starting micro_ros_agent UDP on port ${UDP_PORT}"
    echo "=========================================="
    
    docker run -it --rm \
        --name ${CONTAINER_NAME}-udp \
        --net=host \
        --privileged \
        ${IMAGE_NAME}:${IMAGE_TAG} \
        bash -c "source /opt/ros/humble/setup.bash && ros2 run micro_ros_agent micro_ros_agent udp4 -p ${UDP_PORT}"
    exit 0
fi

echo "=========================================="
echo "Starting container: ${CONTAINER_NAME}"
echo "Image: ${IMAGE_NAME}:${IMAGE_TAG}"
echo "Mounting workspace: ${WORKSPACE_DIR}"
echo ""
echo "Usage tips:"
echo "  - Start DDS Agent (serial): ./run_jetson.sh dds [serial_port] [baud_rate]"
echo "  - Start micro_ros_agent (UDP): ./run_jetson.sh udp [port]"
echo "  - Attach to container: ./run_jetson.sh exec"
echo "=========================================="

docker run -it --rm \
    --name ${CONTAINER_NAME} \
    --net=host \
    --privileged \
    -v /dev:/dev \
    -v "${WORKSPACE_DIR}/drone_navigate:/root/avfl_ws/src/drone_navigate:rw" \
    ${IMAGE_NAME}:${IMAGE_TAG} \
    "$@"
