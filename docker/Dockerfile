# Shared ROS 2 Humble development image for the Mecanumbot path-criteria project.
# Multi-arch: builds natively on amd64 (Linux/WSL) and arm64 (Apple Silicon).
FROM ros:humble

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
ENV DEBIAN_FRONTEND=noninteractive

# Nav2, Gazebo Fortress (ros_gz), mapping, visualization and test tooling.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-ros-gz \
    ros-humble-slam-toolbox \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-foxglove-bridge \
    ros-humble-teleop-twist-keyboard \
    ros-humble-ament-cmake-gtest \
    python3-colcon-common-extensions \
    python3-pip \
    python3-numpy \
    python3-yaml \
    python3-pil \
    gdb \
    && rm -rf /var/lib/apt/lists/*

# Browser-accessible desktop (noVNC) for Gazebo and RViz. Software rendering
# through Mesa works on Docker Desktop for macOS, which has no GPU passthrough.
RUN apt-get update && apt-get install -y --no-install-recommends \
    tigervnc-standalone-server \
    tigervnc-tools \
    novnc \
    websockify \
    openbox \
    xterm \
    mesa-utils \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -m -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
COPY docker/bashrc /etc/mecanumbot.bashrc
RUN chmod +x /usr/local/bin/entrypoint.sh \
    && echo "source /etc/mecanumbot.bashrc" >> /home/${USERNAME}/.bashrc \
    && ln -s /etc/mecanumbot.bashrc /etc/profile.d/zz-mecanumbot.sh \
    && mkdir -p /ws/src /ws/build /ws/install /ws/log \
    && chown -R ${USERNAME}:${USERNAME} /ws

USER ${USERNAME}
WORKDIR /ws
# BASH_ENV also gives non-interactive shells (`docker compose exec dev bash -c`) the ROS setup.
ENV BASH_ENV=/etc/mecanumbot.bashrc \
    DISPLAY=:1 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    ROS_DOMAIN_ID=42

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["sleep", "infinity"]
