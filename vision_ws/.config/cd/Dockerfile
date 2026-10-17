ARG ROS_DISTRO=jazzy

########################################################
# Builder
########################################################

FROM ros:${ROS_DISTRO}-ros-base AS builder

ARG ROS_DISTRO

WORKDIR /app

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Copy package manifests for runtime dependency resolution
COPY packages/levision/ /app/src/levision/

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    mkdir -p /app/install && \
    if find /app/src -name package.xml -print -quit | grep -q .; then \
        rosdep update && \
        apt-get update && \
        rosdep install \
            --from-paths /app/src \
            --ignore-src \
            --rosdistro ${ROS_DISTRO} \
            -r \
            -y && \
        colcon build \
            --merge-install \
            --cmake-args -DCMAKE_BUILD_TYPE=Release; \
    else \
        echo "No ROS packages found in levision; skipping dependency installation and build."; \
    fi

########################################################
# Runtime
########################################################

FROM ros:${ROS_DISTRO}-ros-base AS runtime

ARG ROS_DISTRO

WORKDIR /app

# Install rosdep and supported RMW implementations
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

COPY packages/levision/ /tmp/src/levision/

RUN if find /tmp/src/levision -name package.xml -print -quit | grep -q .; then \
        rosdep update && \
        apt-get update && \
        rosdep install \
            --from-paths /tmp/src/levision \
            --ignore-src \
            --rosdistro ${ROS_DISTRO} \
            -r \
            -y && \
        rm -rf /var/lib/apt/lists/*; \
    else \
        echo "No ROS packages found in levision; skipping runtime dependency installation."; \
    fi && \
    rm -rf /tmp/src

COPY --from=builder /app/install /app/install
COPY vision_ws/.config/cd/entrypoint.sh /entrypoint.sh

RUN chmod +x /entrypoint.sh && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /app/install/setup.bash ]; then source /app/install/setup.bash; fi" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
