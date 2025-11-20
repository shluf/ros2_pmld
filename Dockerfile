# ============================================================================
# Stage 1: Base ROS 2 Humble with system dependencies
# ============================================================================
FROM osrf/ros:humble-desktop-full AS base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV WORKSPACE=/root/ros2_ws

# Install system dependencies and tools
RUN apt-get update && apt-get install -y \
    # Build tools
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    wget \
    curl \
    # OpenCV and vision dependencies
    libopencv-dev \
    # GUI and display support
    x11-apps \
    mesa-utils \
    libgl1-mesa-glx \
    libglib2.0-0 \
    # Network tools for Tello communication
    iproute2 \
    iputils-ping \
    net-tools \
    libasio-dev \
    # Gazebo dependencies
    ros-humble-gazebo-ros-pkgs \
    libgazebo-dev \
    # ROS 2 RMW implementation
    ros-humble-rmw-cyclonedds-cpp \
    # Development utilities
    nano \
    vim \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# Update rosdep
RUN rosdep update

# ============================================================================
# Stage 2: Python dependencies installation
# ============================================================================
FROM base AS dependencies

WORKDIR ${WORKSPACE}

# Install Python packages for perception and control
RUN pip3 install --no-cache-dir --upgrade pip
RUN pip3 install --no-cache-dir --default-timeout=1000 numpy==1.24.3

# Install CPU-only PyTorch
RUN pip3 install --no-cache-dir --ignore-installed torch==2.5.1 torchvision==0.20.1 --index-url https://download.pytorch.org/whl/cpu

# Install Ultralytics (YOLO)
RUN pip3 install --no-cache-dir --default-timeout=1000 ultralytics==8.0.196

# Install TFLite Runtime
RUN pip3 install --no-cache-dir --default-timeout=1000 tflite-runtime==2.14.0

# Install OpenCV Contrib (File Paling Besar - sering timeout disini)
RUN pip3 install --no-cache-dir --default-timeout=1000 opencv-contrib-python==4.8.1.78

# Install MediaPipe
RUN pip3 install --no-cache-dir --default-timeout=1000 mediapipe==0.10.8

# Download YOLO models (for faster first-run)
RUN python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt'); YOLO('yolov8s.pt')" || true

# ============================================================================
# Stage 3: Build workspace with source code
# ============================================================================
FROM dependencies AS builder

# Create workspace structure
RUN mkdir -p ${WORKSPACE}/src

# Copy source code
COPY src/ ${WORKSPACE}/src/

# Install ROS dependencies using rosdep
RUN apt-get update && \
    cd ${WORKSPACE} && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd ${WORKSPACE} && \
    colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --packages-select \
    tello_msgs \
    tello_interfaces \
    tello_perception \
    tello_control \
    gesture_control

# ============================================================================
# Stage 4: Runtime image (minimal)
# ============================================================================
FROM dependencies AS runtime

# Copy built workspace from builder
COPY --from=builder ${WORKSPACE}/install ${WORKSPACE}/install

# Copy source for reference and launch files
COPY src/ ${WORKSPACE}/src/

# Setup entrypoint
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Set working directory
WORKDIR ${WORKSPACE}

# Source ROS 2 and workspace in bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source ${WORKSPACE}/install/setup.bash" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc && \
    echo "alias build='colcon build --symlink-install'" >> ~/.bashrc && \
    echo "alias clean='rm -rf build install log'" >> ~/.bashrc && \
    echo "echo '🚁 ROS 2 Humble Multi-Mode Drone Control System'" >> ~/.bashrc && \
    echo "echo 'Workspace: ${WORKSPACE}'" >> ~/.bashrc

# Expose common ports
# - 11311: ROS Master (if using ROS1 bridge)
# - 7400-7500: DDS Discovery
EXPOSE 11311 7400-7500/udp

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# ============================================================================
# Stage 5: Development image (with full source and dev tools)
# ============================================================================
FROM runtime AS dev

# Install additional development tools
RUN apt-get update && apt-get install -y \
    gdb \
    valgrind \
    clang-format \
    clang-tidy \
    python3-pytest \
    python3-pytest-cov \
    ros-${ROS_DISTRO}-rqt* \
    ros-${ROS_DISTRO}-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Install development Python packages
RUN pip3 install --no-cache-dir \
    pytest \
    pytest-cov \
    black \
    flake8 \
    pylint \
    mypy

# Add helpful aliases for development
RUN echo "alias test='colcon test && colcon test-result --verbose'" >> ~/.bashrc && \
    echo "alias format='black src/'" >> ~/.bashrc && \
    echo "alias lint='flake8 src/'" >> ~/.bashrc

CMD ["bash"]
