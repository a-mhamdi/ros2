FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir /ros2_ws
WORKDIR /ros2_ws
COPY ros2_ws/ .

# launch ros package
CMD ["ros2", "run", "turtlesim", "turtlesim_node"]
