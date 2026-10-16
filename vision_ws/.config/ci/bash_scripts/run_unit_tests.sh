#!/usr/bin/env bash

GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BOLD='\033[1m'
CYAN='\033[0;36m'
RESET='\033[0m'

set -euo pipefail

echo -e "${BOLD}${CYAN}Running tests inside the container...${RESET}"
cd /app

echo -e "${BOLD}${CYAN}Sourcing ROS 2 and workspace setup files...${RESET}"
set +u
source /opt/ros/jazzy/setup.bash
if [ -f install/setup.bash ]; then
    source install/setup.bash
fi
set -u

mapfile -t packages < <(colcon list --names-only)

if [ "${#packages[@]}" -eq 0 ]; then
    echo -e "${BOLD}${YELLOW}No vision packages are present yet. Skipping unit tests.${RESET}"
    exit 0
fi

colcon test \
    --event-handlers console_direct+ \
    --return-code-on-test-failure
