#!/usr/bin/env bash

GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[0;33m'
BOLD='\033[1m'
RESET='\033[0m'

set -euo pipefail

echo -e "${BOLD}${CYAN}Running tests inside the container...${RESET}"
cd /app

echo -e "${BOLD}${CYAN}Sourcing ROS 2 and workspace setup files...${RESET}"
set +u
# first source specific ROS 2 distribution setup file (especially if multiple distributions are installed),
# then source workspace setup file
source /opt/ros/jazzy/setup.bash && source install/setup.bash
set -u

package_exists() {
    colcon list --names-only | grep -q "$1"
}

run_colcon_tests() {
    echo -e "${CYAN}Running colcon tests on $1 package...${RESET}"
    if ! package_exists "$1"; then
        echo -e "${BOLD}${YELLOW}Package $1 does not exist. Skipping tests for this package.${RESET}"
        return
    fi
    colcon test --event-handlers console_direct+ \
        --packages-select "$1" \
        --return-code-on-test-failure
    echo -e "${GREEN}Finished colcon tests on $1 package.${RESET}"
}

run_colcon_tests hiwonder_servo_driver
