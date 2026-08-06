# Unitree Go2 Dog Control

This repo contains code used for controlling the Unitree Go2 robot dog using ROS2.

<!--
## Building and Running the Project
There are two possible ways to run the code in this repository:

1. If you have an Ubuntu 22.04 installation with ROS2 Humble, you can compile and run the code directly on your machine. To do so, follow the instructions in the [Native ROS2 Installation](docs/install/native.md) document.
2. Alternatively, you can run the code inside a development container. To do so, follow the instructions in the [Development Containers](docs/install/dev-container.md) document.

Once you have the requirements set up, you can proceed to the [Compiling the Code](docs/compile.md) guide.
-->

## Structure of this Repository
This repository is a ROS2 workspace.
The code in this repository is seperated into several directories.

- 📂 **config**: Configuration files for experiments
- 📂 **docs**: Documentation and guides (CURRENTLY OUTDATED!!!)
- 📂 **src**: Main source code (organized in ROS2 packages)
    - 📂 **go2_utils**: Library for Go2 robot interaction, used by most other packages
    - 📂 **go2_cli**: Collection of CLI tools for Go2 robot interaction
    - 📂 **experiments**: Loose collection of experiments
    - 📂 **stand_height**: Experiment for letting the robot stand at a fixed height
    - 📂 **mpc_control**: Unfinished implementation of a MPC controller for walking
    - 📂 **basic_motion**: Unfinished implementation of a suite of motion controllers
- 📂 **old**: Old code snippets waiting for deletion
- 📂 **theory**: Notes and scratchpad code

## Quickstart Guide 
You can run this code either in a development container (recommended) or natively on Ubuntu 22.04 with ROS2 Humble. Pick one

### Installing Libraries (Dev Container)
Open the repository in an editor that supports Dev Containers (e.g., _Visual Studio Code_ with the _Dev Containers_ extension), and start the container (this might take a few minutes).

Once the container is ready, you can compile the code using this command:
```bash
colcon build
```

If this succeeds, you can continue to _Running the Code_.

### Installing Libraries (Native)
Make sure you have a working installation of ROS2 Humble (running on Ubuntu 22.04).
Then, import the dependencies of this code by running this command:
```bash
./setup_dependencies.sh
```

Once all dependencies are installed, you can compile the code using this command:
```bash
colcon build
```

If this succeeds, you can continue to _Running the Code_.

### Running the Code
The code in this repository is a collection of ROS2 packages.
As usual, to run them, you first need to activate them by sourcing the setup script:
```bash
source install/setup.bash 
```

Then, you need to select a network configuration to communicate with. 
Choose one of the following (`dds-robot` uses the network interface connected to the robot, while `dds-loopback` allows to communicate with a simulator running on the same machine):
```bash
source dds-robot 
source dds-loopback
```

**Note:** It suffices to run the above commands once when you open your terminal.
The settings will be kept until you close the terminal.

Finally, you can run the code using `ros2`, for example:
```bash
ros2 run go2_cli hl-stand-up
```

### Starting you own Development

Head over to [this page](/docs/getting-started.md) to get started with development.