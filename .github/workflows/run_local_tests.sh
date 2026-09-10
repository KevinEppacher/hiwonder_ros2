 #!/usr/bin/env bash
set -euo pipefail

echo "Running local lint tests using docker. Make sure you have docker installed..."

echo "Running ament_lint_tests job"

export IMAGE_NAME=ros2-container_image-1
export CONTAINER_NAME=ros2-lint-1

docker build -f ci/Dockerfile -t $IMAGE_NAME .

docker run --rm \
    --name $CONTAINER_NAME \
    -v "$(pwd)/ros2_ws:/app:rw" \
    -v "$(pwd)/hiwonder_ros2:/app/src:rw" \
    -v "$(pwd)/ci:/app/ci:ro" \
    $IMAGE_NAME \
    /app/ci/bash_scripts/run_lint_tests.sh

docker run --rm \
    --name $CONTAINER_NAME \
    -v "$(pwd)/ros2_ws:/app:rw" \
    -v "$(pwd)/hiwonder_ros2:/app/src:rw" \
    -v "$(pwd)/ci:/app/ci:ro" \
    $IMAGE_NAME \
    /app/ci/bash_scripts/run_unit_tests.sh
