# Colcon build & workspace management
alias cclean="rm -rf build/ install/ log/"
alias cb="colcon build && source install/setup.bash"
alias cbs="colcon build --symlink-install && source install/setup.bash"
alias s="source install/setup.bash"

# Selective build
alias cbps="colcon build --symlink-install --packages-select"
alias cbps_gdb="colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --packages-select"

# Testing
alias ct="colcon test"
alias ctps="colcon test --packages-select"
alias ctps_cohesion="colcon test --event-handlers console_cohesion+ --packages-select"
alias ct_cohesion="colcon test --event-handlers console_cohesion+"
