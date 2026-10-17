ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO}-ros-core

RUN apt-get update && \
    apt-get install -y \
        python3-pip \
        python3-colcon-common-extensions \
        ros-${ROS_DISTRO}-ament-lint \
        ros-${ROS_DISTRO}-usb-cam && \
    rm -rf /var/lib/apt/lists/*

ENV PIP_BREAK_SYSTEM_PACKAGES=1

COPY vision_ws/.config/ci/requirements.txt /app/requirements.txt
RUN pip install -r /app/requirements.txt

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source install/setup.bash" >> /root/.bashrc

WORKDIR /app
