# Unitree Go2 Dog Control

This repo contains code used for controlling the Unitree Go2 robot dog using ROS2.

## Building and Running the Project
There are two possible ways to run the code in this repository:

1. If you have an Ubuntu 22.04 installation with ROS2 Humble, you can compile and run the code directly on your machine. To do so, follow the instructions in the [Native ROS2 Installation](docs/install/native.md) document.
2. Alternatively, you can run the code inside a development container. To do so, follow the instructions in the [Development Containers](docs/install/dev-container.md) document.

Once you have the requirements set up, you can proceed to the [Compiling the Code](docs/compile.md) guide.



## Structure of this Repository
The code in this repository is seperated into several directories.

- 📂 **docs**: Documentation and guides
- 📂 **include**: Additional headers and libraries used in this repository (e.g. for JSON and CRC)
- 📂 **interfaces**: Interfaces for ROS2 communication
- 📂 **src**: Application source code
    - 📂 **common**: Self-contained utilities that are used in different parts (e.g., the PID controller implementation) 
    - 📂 **controllers**: Robot controllers
        - 📂 **standheight**: Stand height controller
            - 📂 **states**: State machine used in stand height controller
    - 📂 **entrypoints**: `main()` functions for the binaries 
    - 📂 **interface**: Wrappers for robot interaction
    - 📂 **old**: Old code snippets waiting for deletion
- 📂 **theory**: Notes and scratchpad code