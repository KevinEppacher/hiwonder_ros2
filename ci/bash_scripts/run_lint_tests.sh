#!/usr/bin/env bash

set -euo pipefail

GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
RESET='\033[0m'
RED='\033[0;31m'

workspace_root="/app"

echo -e "${BOLD}${CYAN}Building ament_ruff, ament_cmake_ruff packages...${RESET}"
cd "$workspace_root"
colcon build --symlink-install --packages-select \
    ament_ruff \
    ament_cmake_ruff

echo -e "${BOLD}${CYAN}Sourcing ROS 2 and workspace setup files...${RESET}"
set +u
# first source specific ROS 2 distribution setup file (especially if multiple distributions are installed),
# then source workspace setup file
source /opt/ros/jazzy/setup.bash && source install/setup.bash
set -u

run_mypy() {
    echo -e "${CYAN}Performing mypy checks for $1...${RESET}"
    if [ ! -d "/app/src/$1" ]; then
        echo -e "${YELLOW}WARNING: Path /app/src/$1 does not exist.${RESET}"
        return
    fi
    ament_mypy --config /app/ci/linter_configs/mypy.toml "/app/src/$1"
    echo -e "${GREEN}Checked mypy for $1 successfully.${RESET}"
}

run_ruff() {
    echo -e "${CYAN}Performing ruff checks for $1...${RESET}"
    if [ ! -d "/app/src/$1" ]; then
        echo -e "${YELLOW}WARNING: Path /app/src/$1 does not exist.${RESET}"
        return
    fi
    ament_ruff --config /app/ci/linter_configs/ruff.toml "/app/src/$1"
    echo -e "${GREEN}Checked ruff for $1 successfully.${RESET}"
}

run_xmllint() {
    echo -e "${CYAN}Performing xmllint checks for $1...${RESET}"
    if [ ! -d "/app/src/$1" ]; then
        echo -e "${YELLOW}WARNING: Path /app/src/$1 does not exist.${RESET}"
        return
    fi
    ament_xmllint "/app/src/$1"
    echo -e "${GREEN}Checked xmllint for $1 successfully.${RESET}"
}

run_cpplint() {
    echo -e "${CYAN}Performing cpplint checks for $1...${RESET}"
    if [ ! -d "/app/src/$1" ]; then
        echo -e "${YELLOW}WARNING: Path /app/src/$1 does not exist.${RESET}"
        return
    fi
    ament_cpplint "/app/src/$1"
    echo -e "${GREEN}Checked cpplint for $1 successfully.${RESET}"
}

# Linter checks for hiwonder_servo_driver
run_cpplint hiwonder_servo_driver
run_xmllint hiwonder_servo_driver
