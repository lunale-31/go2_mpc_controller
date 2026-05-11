# Unitree Go2 Dog Control

This repo contains code used for controlling the Unitree Go2 robot dog using ROS2.

## Building and Running the Project
There are two possible ways to run the code in this repository:

1. If you have an Ubuntu 22.04 installation with ROS2 Humble, you can compile and run the code directly on your machine. To do so, follow the instructions in the [Native ROS2 Installation](docs/install/native.md) document.
2. Alternatively, you can run the code inside a development container. To do so, follow the instructions in the [Development Containers](docs/install/dev-container.md) document.

Once you have the requirements set up, you can proceed to the [Compiling the Code](docs/compile.md) guide.


## Structure of this Repository
This repository is a ROS2 workspace.
The code in this repository is seperated into several directories.

- 📂 **config**: Configuration files for experiments
- 📂 **docs**: Documentation and guides
- 📂 **src**: Main source code (organized in ROS2 packages)
    - 📂 **go2_cli**: Collection of CLI tools for Go2 robot interaction
    - 📂 **go2_utils**: Library for Go2 robot interaction
    - 📂 **stand_height**: Experiment for letting the robot stand at a fixed height
- 📂 **old**: Old code snippets waiting for deletion
- 📂 **theory**: Notes and scratchpad code

## Quickstart Guide (Dev Container)
Open the repository in an editor that supports Dev Containers (e.g., _Visual Studio Code_ with the _Dev Containers_ extension), and start the container (this might take a few minutes).

Once the container is ready, you can compile the code using this command:
```bash
colcon build
```

## Quickstart Guide (Native)
Make sure you have a working installation of ROS2 Humble (running on Ubuntu 22.04).
Then, import the dependencies of this code by running this command:
```bash
./setup_dependencies.sh
```

Once all dependencies are installed, you can compile the code using this command:
```bash
colcon build
```

