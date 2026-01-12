#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="ardupilot-gazebo-ros2"
IMAGE_TAG="humble-harmonic"

echo "=========================================="
echo "Building Docker image: ${IMAGE_NAME}:${IMAGE_TAG}"
echo "=========================================="

docker build \
    -t ${IMAGE_NAME}:${IMAGE_TAG} \
    -t ${IMAGE_NAME}:latest \
    -f "${SCRIPT_DIR}/dockerfile" \
    "${SCRIPT_DIR}"

if [ $? -eq 0 ]; then
    echo "=========================================="
    echo "Build successful!"
    echo "Image: ${IMAGE_NAME}:${IMAGE_TAG}"
    echo "=========================================="
else
    echo "=========================================="
    echo "Build failed!"
    echo "=========================================="
    exit 1
fi
