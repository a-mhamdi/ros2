FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros \
    nano \
    tmux && \
    rm -rf /var/lib/apt/lists/*

# ----------- BEGIN CONF -----------
RUN cat >> /root/.bashrc << 'EOF'
# Source ROS 2 installation
source /opt/ros/humble/setup.bash

# Add argcomplete initialization to bashrc
eval "$(register-python-argcomplete3 ros2)"

# Check if `tmux` is installed and not already in `tmux`, and then launch it.
if command -v tmux &> /dev/null && [ -n "$PS1" ] && [[ ! "$TERM" =~ screen ]] && [[ ! "$TERM" =~ tmux ]] && [ -z "$TMUX" ]; then
exec tmux
fi
EOF
# ----------- END CONF -----------

RUN mkdir /ros2_ws
WORKDIR /ros2_ws
COPY ros2_ws/ .

# Open BASH
CMD ["bash"]
