ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO}-ros-core

########################################################
# Custom installations
########################################################

# Install python3-pip for installing Python packages with pip
RUN apt update && \
    apt upgrade -y && \
    apt install -y python3-pip

# We do not care about breaking system packages in this container
ENV PIP_BREAK_SYSTEM_PACKAGES=1

# Install python packages from requirements.txt
COPY motion_ws/.config/ci/requirements.txt /app/requirements.txt
RUN pip install -r /app/requirements.txt

# Install ros ament lint packages
RUN apt update && apt install -y \
    ros-${ROS_DISTRO}-ament-lint

# Install colcon, since ros core does not come with it
RUN apt update && apt install -y \
    python3-colcon-common-extensions

# Install DDS middleware
RUN apt update && apt install -y \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp

# Install USB camera support
RUN apt update && apt install -y \
    ros-${ROS_DISTRO}-usb-cam

########################################################
# End Custom installations
########################################################

RUN apt update && apt upgrade -y

# Source ROS setup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source install/setup.bash" >> ~/.bashrc

# Configure CycloneDDS as the default RMW implementation
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

WORKDIR /app
