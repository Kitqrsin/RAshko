FROM ros:humble

# Windows-side ROS, visualization, planning, and camera dependencies.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-robot-state-publisher \
    ros-humble-moveit \
    ros-humble-camera-calibration \
    python3-tk \
    libatlas-base-dev \
    v4l-utils \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# LLM HANDOFF: keep this CPU index explicit. The default resolver installs
# several gigabytes of CUDA libraries that Docker Desktop does not use here.
RUN pip3 install --no-cache-dir \
    --index-url https://download.pytorch.org/whl/cpu \
    torch \
    torchvision

# NumPy stays below 2 for ROS Humble binary-extension compatibility. Keep one
# OpenCV distribution; installing both GUI and headless wheels corrupts cv2.
RUN pip3 install --no-cache-dir \
    "Flask>=3.0,<4.0" \
    "numpy<2" \
    Pillow \
    "Werkzeug>=2.0" \
    "pytest>=7.0,<8.0" \
    "packaging>=24.0,<25.0" \
    "opencv-contrib-python>=4.6,<4.12" \
    ultralytics \
    py_trees \
    && pip3 uninstall -y opencv-python \
    && pip3 install --no-cache-dir --force-reinstall --no-deps \
        "opencv-contrib-python>=4.6,<4.12"

WORKDIR /workspace

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> ~/.bashrc

CMD ["/bin/bash"]
