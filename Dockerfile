FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-tf2-ros \
    ros-humble-pointcloud-to-laserscan \
    python3-pip \
    git \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x /lib/cyclonedds && \
    mkdir -p /lib/cyclonedds/build /lib/cyclonedds/install && \
    cd /lib/cyclonedds/build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/lib/cyclonedds/install && \
    cmake --build . --target install

RUN git clone https://github.com/unitreerobotics/unitree_sdk2_python.git /lib/unitree_sdk2_python && \
    touch /lib/unitree_sdk2_python/unitree_sdk2py/b2/__init__.py && \
    touch /lib/unitree_sdk2_python/unitree_sdk2py/comm/__init__.py && \
    touch /lib/unitree_sdk2_python/unitree_sdk2py/h1/__init__.py && \
    mkdir -p /lib/unitree_sdk2_python/unitree_sdk2py/g1 && \
    touch /lib/unitree_sdk2_python/unitree_sdk2py/g1/__init__.py && \
    touch /lib/unitree_sdk2_python/unitree_sdk2py/g1/loco/__init__.py && \
    touch /lib/unitree_sdk2_python/unitree_sdk2py/g1/arm/__init__.py && \
    touch /lib/unitree_sdk2_python/unitree_sdk2py/g1/audio/__init__.py && \
    cd /lib/unitree_sdk2_python && \
    export CYCLONEDDS_HOME=/lib/cyclonedds/install && \
    pip3 install --no-cache-dir .

WORKDIR /opt/ws/src
COPY . ./g1_slam

WORKDIR /opt/ws
RUN (rosdep init 2>/dev/null || true) && \
    rosdep update && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install && \
    rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /opt/ws/install/setup.bash" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc

CMD ["bash"]
