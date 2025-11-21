FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros \
    libqt5gui5 \
    qt5-qmake \
    libqt5core5a \
    tmux && \
    rm -rf /var/lib/apt/lists/*

# Add argcomplete initialization to bashrc
RUN echo 'eval "$(register-python-argcomplete3 ros2)"' >> /root/.bashrc

RUN mkdir /ros2_ws
WORKDIR /ros2_ws
COPY ros2_ws/ .

# Launch ROS 2 `turtlesim` package
CMD ["ros2", "run", "turtlesim", "turtlesim_node"]
