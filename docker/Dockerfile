ARG ROS_DISTRO=jazzy

FROM docker.io/osrf/ros:${ROS_DISTRO}-desktop-full

ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN if ! getent passwd "${USER_UID}" >/dev/null 2>&1; then \
        groupadd --gid "${USER_GID}" "${USERNAME}" && \
        useradd -s /bin/bash --uid "${USER_UID}" --gid "${USER_GID}" -m "${USERNAME}"; \
    else \
        echo "UID ${USER_UID} is already in use; choose a different USER_UID" >&2; exit 1; \
    fi

RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME

RUN sudo apt update && sudo apt upgrade -y

########################################################
# Custom installations (Nav2 focused)
########################################################


########################################################
# End Custom installations
########################################################

RUN sudo apt update && sudo apt upgrade -y

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source install/setup.bash" >> ~/.bashrc
RUN echo "cd /app && source .bash_aliases" >> ~/.bashrc

WORKDIR /app
