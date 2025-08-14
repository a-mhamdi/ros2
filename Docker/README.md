# Running ROS2 with Docker

This repository offers a convenient and ready-to-run solution for using **ROS2 Humble** on various platforms via Docker. You can launch the `turtlesim_node` by running the provided script or using Docker Compose.

## Using the Script

To see a basic example in action, simply run the `turtlesim_node` by executing the following script:

```bash
./script.sh
```

## Using Docker Compose

Alternatively, you can use `docker-compose` to manage the containers. This is especially useful for more complex setups with multiple services. To run the same example with Docker Compose, use this command:

```bash
docker-compose up -d
```

This will build and run the necessary containers as defined in the [docker-compose.yml](https://raw.githubusercontent.com/a-mhamdi/ros2/refs/heads/main/docker-compose.yml) file.

**GitHub Actions** build and push this image to [dockerhub](https://hub.docker.com/). Every update is available at [abmhamdi/ros2](https://hub.docker.com/repository/docker/abmhamdi/ros2).

[![Docker Pulls](https://img.shields.io/docker/pulls/abmhamdi/ros2)](https://hub.docker.com/r/abmhamdi/ros2)
[![Docker Stars](https://img.shields.io/docker/stars/abmhamdi/ros2)](https://hub.docker.com/r/abmhamdi/ros2)
