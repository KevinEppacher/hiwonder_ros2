#!/usr/bin/env bash

set -euo pipefail

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
RESET='\033[0m'

workspace_root="/app"
cd "$workspace_root"

echo
set +u
source /opt/ros/jazzy/setup.bash
set -u

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

mapfile -t package_lines < <(colcon list)

if [ "${#package_lines[@]}" -eq 0 ]; then
    echo -e "${YELLOW}No vision packages are present yet. Skipping package lint checks.${RESET}"
    exit 0
fi

for line in "${package_lines[@]}"; do
    read -r package package_path _ <<<"$line"
    package_dir="$workspace_root/$package_path"

    run_linter xmllint "$package" \
        ament_xmllint "$package_dir"

    if find "$package_dir" -type f \( -name '*.c' -o -name '*.cc' -o -name '*.cpp' -o -name '*.h' -o -name '*.hpp' \) -print -quit | grep -q .; then
        run_linter cpplint "$package" \
            ament_cpplint "$package_dir"
    fi
done

echo
echo -e "${BOLD}${GREEN}✓ All linter checks passed${RESET}"
