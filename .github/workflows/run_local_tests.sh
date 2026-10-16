 #!/usr/bin/env bash
set -euo pipefail

echo "Running local lint tests using docker. Make sure you have docker installed..."

echo "Running ament_lint_tests job"

REPO_ROOT="$(git -C "$(dirname "${BASH_SOURCE[0]}")/../.." rev-parse --show-toplevel)"
cd "$REPO_ROOT"

export IMAGE_NAME=ros2-container_image-1
export CONTAINER_NAME=ros2-lint-1

docker build -f motion_ws/.config/ci/Dockerfile -t "$IMAGE_NAME" .

docker run --rm \
    --name "$CONTAINER_NAME" \
    -v "$(pwd)/motion_ws:/app:rw" \
    -v "$(pwd)/lemotion:/app/src:rw" \
    -v "$(pwd)/motion_ws/.config/ci:/app/ci:ro" \
    "$IMAGE_NAME" \
    /app/ci/bash_scripts/run_lint_tests.sh

docker run --rm \
    --name "$CONTAINER_NAME" \
    -v "$(pwd)/motion_ws:/app:rw" \
    -v "$(pwd)/lemotion:/app/src:rw" \
    -v "$(pwd)/motion_ws/.config/ci:/app/ci:ro" \
    "$IMAGE_NAME" \
    /app/ci/bash_scripts/run_unit_tests.sh
