#!/usr/bin/env bash

set -euo pipefail

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
RESET='\033[0m'

workspace_root="/app"

echo -e "${BOLD}${CYAN}Building linter packages...${RESET}"
cd "$workspace_root"

build_log="$(mktemp)"

echo
echo -e "${BOLD}${CYAN}Sourcing ROS 2 and workspace...${RESET}"

set +u
source /opt/ros/jazzy/setup.bash
set -u

echo -e "${GREEN}✓ Workspace sourced${RESET}"
echo

check_path() {
    if [ ! -d "$workspace_root/src/$1" ]; then
        echo -e "${YELLOW}⚠ Package $1 does not exist. Skipping.${RESET}"
        return 1
    fi
}

run_linter() {
    local linter="$1"
    local package="$2"
    shift 2

    local log_file
    log_file="$(mktemp)"

    if "$@" >"$log_file" 2>&1; then
        echo -e "${GREEN}✓ ${package}: ${linter}${RESET}"
        rm -f "$log_file"
    else
        echo -e "${BOLD}${RED}✗ ${package}: ${linter} failed${RESET}"
        echo
        cat "$log_file"
        rm -f "$log_file"
        return 1
    fi
}

run_xmllint() {
    check_path "$1" || return

    run_linter xmllint "$1" \
        ament_xmllint "$workspace_root/src/$1"
}

run_cpplint() {
    check_path "$1" || return

    run_linter cpplint "$1" \
        ament_cpplint "$workspace_root/src/$1"
}

# Linter checks for hiwonder_servo_driver
run_cpplint hiwonder_servo_driver
run_xmllint hiwonder_servo_driver

# Linter checks for lerobot_cpp
run_cpplint lerobot_cpp
run_xmllint lerobot_cpp

# Linter checks for hiwonder_lerobot_cpp
run_cpplint hiwonder_lerobot_cpp
run_xmllint hiwonder_lerobot_cpp

# Linter checks for lerobot_ros2_control
run_cpplint lerobot_ros2_control
run_xmllint lerobot_ros2_control

echo
echo -e "${BOLD}${GREEN}✓ All linter checks passed${RESET}"
