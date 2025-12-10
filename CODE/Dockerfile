# ==============================================================================
# Base Image & Environment Setup
# ==============================================================================
FROM ros:humble-ros-base-jammy

# Avoid interactive install issues
ENV DEBIAN_FRONTEND=noninteractive
ENV ZED_DIR=/usr/local/zed

# User configuration
ARG UID=1000
ARG GID=1001
ARG USERNAME=tejas

# ==============================================================================
# System Updates & Basic Dependencies
# ==============================================================================
RUN apt-get update && \
    apt-get install -y \
    wget \
    lsb-release \
    gnupg \
    zstd \
    sudo \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# ==============================================================================
# Display & NVIDIA Dependencies
# ==============================================================================
RUN apt-get update && apt-get install -y \
    # X11 and display
    x11-apps \
    x11-utils \
    libx11-dev \
    libxext-dev \
    libxrender-dev \
    libxtst-dev \
    # OpenGL and graphics
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libglu1-mesa \
    mesa-utils \
    # NVIDIA OpenGL
    libnvidia-gl-525 \
    # Additional graphics libraries
    libglvnd0 \
    libglx0 \
    libglvnd-dev \
    # OGRE for RViz2
    libogre-1.12-dev \
    && rm -rf /var/lib/apt/lists/*

# ==============================================================================
# ZED SDK Installation
# ==============================================================================
RUN wget -q https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/5.1/ZED_SDK_Ubuntu22_cuda13.0_tensorrt10.13_v5.1.1.zstd.run -O /tmp/ZED_SDK.run && \
    chmod +x /tmp/ZED_SDK.run && \
    /tmp/ZED_SDK.run -- silent && \
    rm /tmp/ZED_SDK.run

ENV CMAKE_PREFIX_PATH=/usr/local/zed:$CMAKE_PREFIX_PATH

# ==============================================================================
# User Setup
# ==============================================================================
RUN groupadd -g $GID $USERNAME && \
    useradd -m -u $UID -g $GID -s /bin/bash $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    usermod -aG zed,video,audio $USERNAME

# Set ZED permissions
RUN chmod -R g+r /usr/local/zed && \
    chmod -R g+x /usr/local/zed/lib && \
    chown -R $USERNAME:$USERNAME /usr/local/lib/python3.10/dist-packages

# ==============================================================================
# ROS 2 Dependencies
# ==============================================================================
RUN rosdep update

# Copy source files for dependency resolution
COPY --chown=$USERNAME:$USERNAME ./ros_ws/src /tmp/workspace/src

# Extract package.xml files
RUN mkdir -p /tmp/pkg_xmls && \
    find /tmp/workspace/src -name "package.xml" -exec cp --parents {} /tmp/pkg_xmls \;

# Install ROS dependencies
RUN rosdep install --from-paths /tmp/pkg_xmls --ignore-src -r -y && \
    rm -rf /tmp/workspace /tmp/pkg_xmls

RUN apt-get update && apt-get install -y ros-humble-rviz2 \
                                            ros-humble-cv-bridge

# ==============================================================================
# Python Dependencies
# ==============================================================================
RUN pip install --upgrade \
    numpy==1.26.4 \
    open3d==0.19.0 \
    scipy==1.15.3 \
    opencv-python==4.11.0.86 \
    opencv-contrib-python==4.11.0.86 \
    PyYAML==6.0.3

# ==============================================================================
# CUDA Environment Setup
# ==============================================================================
ENV CUDA_HOME=/usr/local/cuda-13.0
ENV PATH=/usr/local/cuda-13.0/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/cuda-13.0/lib64:${LD_LIBRARY_PATH}

# ==============================================================================
# Workspace & User Configuration
# ==============================================================================
WORKDIR /workspace

# Setup .bashrc for ROS environment
RUN echo "source /opt/ros/humble/setup.bash" > /home/$USERNAME/.bashrc && \
    echo "if [ -f /workspace/ros_ws/install/setup.bash ]; then source /workspace/ros_ws/install/setup.bash; fi" >> /home/$USERNAME/.bashrc && \
    chown $USERNAME:$USERNAME /home/$USERNAME/.bashrc

# Create .stereolabs directories
RUN mkdir -p /home/$USERNAME/.stereolabs/settings /home/$USERNAME/.stereolabs/resources && \
    chown -R $USERNAME:$USERNAME /home/$USERNAME/.stereolabs

# Switch to non-root user
USER $USERNAME

CMD ["bash"]
