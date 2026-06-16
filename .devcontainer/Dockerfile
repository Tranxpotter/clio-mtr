# ROS 2 Humble Development Container for Go2 MID360
# CUDA 12.1 + Ubuntu 22.04 (Jammy)
FROM nvidia/cuda:12.1.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    locales \
    && sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen \
    && locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

ENV CUDA_HOME=/usr/local/cuda
ENV PATH=${CUDA_HOME}/bin:${PATH}
ENV LD_LIBRARY_PATH=${CUDA_HOME}/lib64:${LD_LIBRARY_PATH}

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && rm -rf /var/lib/apt/lists/*

ARG ROS_VERSION
ARG WORKSPACE_NAME

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_VERSION}-desktop \
    ros-${ROS_VERSION}-ros2bag \
    ros-${ROS_VERSION}-rmw-cyclonedds-cpp \
    ros-${ROS_VERSION}-rosidl-generator-dds-idl \
    ros-${ROS_VERSION}-pcl-ros \
    ros-${ROS_VERSION}-pcl-conversions \
    ros-${ROS_VERSION}-rviz2 \
    ros-${ROS_VERSION}-rviz-common \
    ros-${ROS_VERSION}-rviz-imu-plugin \
    ros-${ROS_VERSION}-slam-toolbox \
    ros-${ROS_VERSION}-compressed-image-transport \
    ros-${ROS_VERSION}-pointcloud-to-laserscan \
    ros-${ROS_VERSION}-nav2-util \
    ros-${ROS_VERSION}-nav2-map-server \
    ros-${ROS_VERSION}-nav2-bringup \
    ros-${ROS_VERSION}-nav2-amcl \
    ros-${ROS_VERSION}-navigation2 \
    ros-${ROS_VERSION}-twist-mux \
    ros-${ROS_VERSION}-topic-tools \
    ros-${ROS_VERSION}-sophus \
    ros-${ROS_VERSION}-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    xvfb \
    x11vnc \
    novnc \
    websockify \
    fluxbox \
    supervisor \
    xauth \
    x11-utils \
    x11-apps \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-numpy \
    python3-pip \
    libeigen3-dev \
    libpcl-dev \
    libopencv-dev \
    libboost-all-dev \
    libtbb-dev \
    libyaml-cpp-dev \
    libaprutil1-dev \
    libx11-6 \
    libx11-xcb1 \
    libxcb1 \
    libxrender1 \
    libxrandr2 \
    libxi6 \
    ffmpeg \
    libx264-dev \
    portaudio19-dev \
    ros-${ROS_VERSION}-foxglove-bridge \
    ros-${ROS_VERSION}-ros-testing \
    ros-${ROS_VERSION}-rqt-graph \
    ros-${ROS_VERSION}-rqt-plot \
    ros-${ROS_VERSION}-rqt-tf-tree \
    ros-${ROS_VERSION}-rqt-robot-monitor \
    ros-${ROS_VERSION}-rqt-bag \
    ros-${ROS_VERSION}-rqt-common-plugins \
    && rm -rf /var/lib/apt/lists/*

ARG USERNAME
ARG USER_UID
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd -m -u $USER_UID -g $USER_GID -s /bin/bash $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

RUN mkdir -p /root/.cache/pip && \
    chown -R iwintern:iwintern /root/.cache/pip

COPY requirements.txt /tmp/requirements.txt
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    pip3 install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir -r /tmp/requirements.txt && \
    rm /tmp/requirements.txt

ARG WORKSPACE_NAME
WORKDIR /workspaces/${WORKSPACE_NAME}

ARG USERNAME
RUN mkdir -p /root/.vnc /home/$USERNAME/.vnc /var/log/supervisor /var/run/supervisor && \
    mkdir -p /tmp/.X11-unix && chmod 1777 /tmp/.X11-unix && \
    chown -R $USERNAME:$USERNAME /home/$USERNAME && \
    mkdir -p /workspaces/${WORKSPACE_NAME} && \
    chown -R $USERNAME:$USERNAME /workspaces

# Copy configuration files
COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf
COPY scripts/entrypoint.sh /usr/local/bin/
COPY scripts/display-utils.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/entrypoint.sh /usr/local/bin/display-utils.sh

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    ARCH=$(dpkg --print-architecture) && \
    if [ "$ARCH" = "amd64" ] || [ "$ARCH" = "arm64" ]; then \
    curl -fsSL -o /tmp/virtualgl.deb \
    "https://github.com/VirtualGL/virtualgl/releases/download/3.1.1/virtualgl_3.1.1_${ARCH}.deb" \
    && dpkg -i /tmp/virtualgl.deb && rm /tmp/virtualgl.deb \
    && apt-get install -f -y && rm -rf /var/lib/apt/lists/* \
    || echo "VirtualGL install failed (network/package issue), continuing without GPU acceleration"; \
    else \
    echo "VirtualGL not available for arch: $ARCH, skipping (software GL fallback will be used)"; \
    fi

RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    pip3 install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir --ignore-installed sympy

# Copy post-start configuration script
COPY scripts/post-start.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/post-start.sh

ARG DISPLAY_MODE
ENV DISPLAY_MODE=${DISPLAY_MODE}

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["supervisord", "-n"]