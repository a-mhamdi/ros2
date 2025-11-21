FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros \
    libqt5gui5 \
    qt5-qmake \
    libqt5core5a \
    nano \
    tmux && \
    rm -rf /var/lib/apt/lists/*

# Add argcomplete initialization to bashrc
RUN echo 'eval "$(register-python-argcomplete3 ros2)"' >> /root/.bashrc

# Check if `tmux` is installed and not already in `tmux`, and then launch it.
RUN cat >> /root/.bashrc << 'EOF'
if command -v tmux &> /dev/null && [ -n "$PS1" ] && [[ ! "$TERM" =~ screen ]] && [[ ! "$TERM" =~ tmux ]] && [ -z "$TMUX" ]; then
  exec tmux
fi
EOF

RUN mkdir /ros2_ws
WORKDIR /ros2_ws
COPY ros2_ws/ .

# Launch ROS 2 `turtlesim` package
CMD ["ros2", "run", "turtlesim", "turtlesim_node"]
