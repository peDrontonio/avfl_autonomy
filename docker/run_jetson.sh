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


echo "=========================================="
echo "Starting container: ${CONTAINER_NAME}"
echo "Image: ${IMAGE_NAME}:${IMAGE_TAG}"
echo "Mounting workspace: ${WORKSPACE_DIR}"
echo ""
echo "Usage tips:"
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
