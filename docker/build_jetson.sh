#!/bin/bash
# ============================================
# AVFL AUTONOMY - JETSON BUILD SCRIPT
# Builds Docker image for real drone operations
# ============================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="avfl-jetson"
IMAGE_TAG="humble"

echo "=========================================="
echo "Building Jetson Docker image: ${IMAGE_NAME}:${IMAGE_TAG}"
echo "For real drone operations (no Gazebo/SITL)"
echo "=========================================="

docker build \
    -t ${IMAGE_NAME}:${IMAGE_TAG} \
    -t ${IMAGE_NAME}:latest \
    -f "${SCRIPT_DIR}/dockerfile.jetson" \
    "${SCRIPT_DIR}"

if [ $? -eq 0 ]; then
    echo "=========================================="
    echo "Build successful!"
    echo "Image: ${IMAGE_NAME}:${IMAGE_TAG}"
    echo ""
    echo "To run the container, use:"
    echo "  ./run_jetson.sh"
    echo "=========================================="
else
    echo "=========================================="
    echo "Build failed!"
    echo "=========================================="
    exit 1
fi
