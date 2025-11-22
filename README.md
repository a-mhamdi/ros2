# ROS 2: Humble Hawksbill

> [!NOTE]
> You can either follow the steps detailed [here](https://docs.ros.org/en/humble/Installation.html) for local installation or use the provided `Docker` image for a containerized environment.

<!--
## Prerequisites

To begin, you need to source the **ROS 2** installation and the current shared package directory. This command sets up your environment to use the **ROS 2** tools:
```bash
source /opt/ros/humble/setup.bash
```
-->

## Docker Setup

Codes run on top of a `Docker` image, ensuring a consistent and reproducible environment. 

> [!IMPORTANT]
>
> You will need to have `Docker` installed on your machine. You can download it from the [Docker website](https://hub.docker.com).

To run the code, you will need to first pull the `Docker` image by running the following command:
```bash
docker pull abmhamdi/ros2
```

This may take a while, as the image contains projects' source code, **ROS 2**, and all necessary system dependencies.

## How to control the containers

* Start the containers in detached mode:
```bash
docker compose up -d
``` 
* Open an interactive `bash` session inside the `ros2` service:
```bash
docker compose exec -it ros2 bash
``` 
You can build a specific package by using the `colcon` build command and specifying the package name:
```bash
colcon build --packages-select <package_name>
```
* Stop and remove the containers:
```bash
docker compose down
``` 

## License
This project is licensed under the MIT License - see the [LICENSE](https://raw.githubusercontent.com/a-mhamdi/ros2/refs/heads/main/LICENSE) file for details.
