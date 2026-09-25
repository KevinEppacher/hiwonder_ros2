ARG ROS_DISTRO=jazzy

########################################################
# Builder
########################################################

FROM ros:${ROS_DISTRO}-ros-base AS builder

ARG ROS_DISTRO

WORKDIR /app

# Install build tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Copy ROS 2 packages
COPY hiwonder_ros2/ /app/src/

# Install package dependencies
RUN rosdep update && \
    apt-get update && \
    rosdep install \
        --from-paths /app/src \
        --ignore-src \
        --rosdistro ${ROS_DISTRO} \
        -r \
        -y && \
    rm -rf /var/lib/apt/lists/*

# Build workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build \
        --merge-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release


########################################################
# Runtime
########################################################

FROM ros:${ROS_DISTRO}-ros-base AS runtime

ARG ROS_DISTRO

WORKDIR /app

# Install rosdep
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Copy package manifests for runtime dependency resolution
COPY hiwonder_ros2/ /tmp/src/

# Install runtime dependencies
RUN rosdep update && \
    apt-get update && \
    rosdep install \
        --from-paths /tmp/src \
        --ignore-src \
        --rosdistro ${ROS_DISTRO} \
        -r \
        -y && \
    rm -rf /var/lib/apt/lists/* \
        /tmp/src

# Copy compiled workspace
COPY --from=builder /app/install /app/install

# Source workspace automatically
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /app/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
