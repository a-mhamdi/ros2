# ROS2: Humble Hawksbill

To get things rolling, give this file a look at first: [https://gist.github.com/a-mhamdi/5793a17dcdf784da8b01e1847f6f7d1d](https://gist.github.com/a-mhamdi/5793a17dcdf784da8b01e1847f6f7d1d)

## Prerequisites

> [!NOTE]
> You can either follow the steps below for local installation or use the provided Docker image for a containerized environment.

Source the **ROS** installation and current shared package directory.

```bash
source install/setup.bash
```
Compile the packages:
1. `listen_talk_pkg`
```bash
colcon build --packages-select listen_talk_pkg
```
2. `robot`
```bash
colcon build --packages-select robot
```

## Docker Setup
Codes run on top of a `Docker` image, ensuring a consistent and reproducible environment. 

> [!IMPORTANT]
>
> You will need to have `Docker` installed on your machine. You can download it from the [Docker website](https://hub.docker.com).

To run the code, you will need to first pull the `Docker` image by running the following command:

```zsh
docker pull abmhamdi/ros
```

This may take a while, as it will download and install all necessary dependencies.

## How to control the containers:

* ```docker-compose up -d``` starts the container in detached mode
* ```docker-compose down``` stops and destroys the container

<!-- Services can be run by typing the command `docker-compose up`. This will start the `Jupyter Lab` on [http://localhost:2468](http://localhost:2468), and you should be able to use `Python` from within the notebook by starting a new `Python` notebook. You can parallelly start `Marimo` on [http://localhost:1357](http://localhost:1357). -->

## License
This project is licensed under the MIT License - see the [LICENSE](https://raw.githubusercontent.com/a-mhamdi/nlp/refs/heads/main/LICENSE) file for details.
