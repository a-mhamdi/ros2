FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros && \
    rm -rf /var/lib/apt/lists/*

# launch ros package
CMD ["ros2", "run", "turtlesim", "turtlesim_node"]
