#!/usr/bin/bash

# Allow any user to connect to the X server
xhost + # @ each startup

docker run -it                                  \
    --privileged                                \
    --net=host                                  \
    --device /dev/dri                           \
    --env="DISPLAY"                             \
    --env="QT_X11_NO_MITSHM=1"                  \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    abmhamdi/ros2
