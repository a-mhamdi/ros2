# ROS2: Humble Hawksbill

## Prerequisites

> [!NOTE]
> You can either follow the steps below for local installation or use the provided Docker image for a containerized environment.

To begin, you need to source the ROS 2 installation and the current shared package directory. This command sets up your environment to use the *ROS2* tools:

```bash
source install/setup.bash
```

Once your environment is set up, you can build a specific package by using the `colcon` build command and specifying the package name:

```bash
colcon build --packages-select <package_name>
```

## Docker Setup
Codes run on top of a `Docker` image, ensuring a consistent and reproducible environment. 

> [!IMPORTANT]
>
> You will need to have `Docker` installed on your machine. You can download it from the [Docker website](https://hub.docker.com).

To run the code, you will need to first pull the `Docker` image by running the following command:

```zsh
docker pull abmhamdi/ros2
```

This may take a while, as it will download and install all necessary dependencies.

## How to control the containers:

* ```docker-compose up -d``` starts the container in detached mode
* ```docker-compose down``` stops and destroys the container

## License
This project is licensed under the MIT License - see the [LICENSE](https://raw.githubusercontent.com/a-mhamdi/ros2/refs/heads/main/LICENSE) file for details.
