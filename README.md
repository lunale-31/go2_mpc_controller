# Unitree Go2 Dog Control

This repo contains code used for controlling the Unitree Go2 robot dog using ROS2.

## Building and Running the Project
There are two possible ways to run the code in this repository:

1. If you have an Ubuntu 22.04 installation with ROS2 Humble, you can compile and run the code directly on your machine. To do so, follow the instructions in the _Native ROS2 Installation_ Section.
2. Alternatively, you can run the code inside a development container. To do so, follow the instructions in the _Development Containers_ Section.

Once you have the requirements set up, you can proceed to the _Compiling the Code_ Section.

### Native ROS2 Installation
To run the code natively on your computer, you need to have Ubuntu 22.04 with ROS2 Humble installed on your machine. 
A guide for installing ROS2 Humble can be found [in its documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
On top of the standard ROS2 installation, you will need to install Unitree's ROS2 library (official documentation [here](https://github.com/unitreerobotics/unitree_ros2/blob/master/README.md)).
In the remainder of this section, you can find the required network configuration and an installation guide for the library.
Once you have set up this, you can proceed to the _Compiling the Code_ Section.

#### Network Connection
To communicate with the robot, you first need to configure the network interface connected to the robot.
Use the following settings for the network interface connected to the robot:
- IPv4 Method: `Manual`
- IPv4 Address: `192.168.123.99`
- IPv4 Netmask: `255.255.255.0` (or `24`, depending on the system)


#### Building Unitree's ROS2 Library
Start by opening a terminal and install the dependencies of ROS2:
```bash
sudo apt update
sudo apt install build-essential git cmake gedit ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-generator-dds-idl libyaml-cpp-dev
```

Then, clone Unitree's ROS2 repository into your home directory:
```bash
git clone https://github.com/unitreerobotics/unitree_ros2 ~/unitree_ros2
```

Next, enter the cloned repository and run the following commands to fetch and build the CycloneDDS communication middleware used by Unitree.
```bash
cd ~/unitree_ros2/cyclonedds_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b humble
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x 
cd ..
colcon build --packages-select cyclonedds
```

Then, compile the Unitree ROS2 library itself:
```bash
. /opt/ros/humble/setup.bash
colcon build
```

Finally, add the library to your profile configuration to load it automatically when opening a terminal:
```bash
gedit ~/.profile
```

In the text editor, append the following lines to the end of the file and save it:
```
echo "Setting up the Unitree ROS2 environment"
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Now, close both the editor and the terminal.

#### Testing the Connection
Open a new terminal. 
If you have one open from previous steps, close it and open a new one to load the changed profile configuration!

There, check that the network interface is configured correctly by running the following command:
```bash
ip r | grep 192.168.123
```
If everything is configured correctly, you will receive this (or a similar) output:
```
192.168.123.0/24 dev eth0 proto kernel scope link src 192.168.123.99 metric 100 
```
Note that depending on your hardware, the interface name (here: `eth0`) might differ (e.g., `ethX` or `enpXsY`, where `X` and `Y` are numeric).
Remember or write that name down for troubleshooting.

Now, you can query the list of ROS 2 topics provided by the robot dog. 
Run the following command in a new terminal. 

```bash
ros2 topic list
```
If everything is working as intended, you will receive a long list of ROS 2 topics, indicating that you are connected to the robot:
```
/api/arm/request
/api/assistant_recorder/request
/api/assistant_recorder/response
/api/audiohub/request
/api/audiohub/response
/api/bashrunner/request
/api/bashrunner/response
/api/config/request
/api/config/response
[...]
/uwbstate
/uwbswitch
/videohub/inner
/webrtcreq
/webrtcres
/wirelesscontroller
/wirelesscontroller_unprocessed
/xfk_webrtcreq
/xfk_webrtcres
```

If this is not the case, try explicitly selecting the network interface by running the following command before re-running `ros2 topic list`. 
You might have to replace `eth0` by the name of the network interface connected to the robot.
```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'
```

Once you are connected to the robot, you can proceed to the _Compiling the Code_ Section.


### Development Containers
The code found in this repository can be executed inside a development container. 
The sources required for setting it up are provided in the `.devcontainer/` directory. 
We recommend the use of Visual Studio Code with the _Dev Containers_ extension for this.
You can find details on using development containers in VS Code in [their documentation](https://code.visualstudio.com/docs/devcontainers/containers).

Before creating the container, you need to configure the network interface connected to the robot.
Use the following settings for the network interface connected to the robot:
- IPv4 Method: `Manual`
- IPv4 Address: `192.168.123.99`
- IPv4 Netmask: `255.255.255.0` (or `24`, depending on the system)

Now, build the container and enter a shell in it (e.g., by opening a terminal in VS Code).
Check that the network interface is available in the container by running the following command:
```bash
ip r | grep 192.168.123
```
If everything is configured correctly, you will receive this (or a similar) output:
```
192.168.123.0/24 dev eth0 proto kernel scope link src 192.168.123.99 metric 100 
```
Note that depending on your hardware, the interface name (here: `eth0`) might differ (e.g., `ethX` or `enpXsY`, where `X` and `Y` are numeric). 
Remember or write that name down for troubleshooting.

Now, you can query the list of ROS 2 topics provided by the robot dog. 
Run the following command.
```bash
ros2 topic list
```
If everything is working as intended, you will receive a long list of ROS 2 topics, indicating that you are connected to the robot:
```
/api/arm/request
/api/assistant_recorder/request
/api/assistant_recorder/response
/api/audiohub/request
/api/audiohub/response
/api/bashrunner/request
/api/bashrunner/response
/api/config/request
/api/config/response
[...]
/uwbstate
/uwbswitch
/videohub/inner
/webrtcreq
/webrtcres
/wirelesscontroller
/wirelesscontroller_unprocessed
/xfk_webrtcreq
/xfk_webrtcres
```

If this is not the case, try explicitly selecting the network interface by running the following command before re-running `ros2 topic list`. 
You might have to replace `eth0` by the name of the network interface connected to the robot.
```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'
```

Once you are connected to the robot, you can proceed to the _Compiling the Code_ Section.

### Compiling the Code
For this guide, we assume that you have already cloned the repository to you machine.
Open a terminal and navigate into the repository (`cd /path/to/the/repo/`).

There, create a new folder `build` and enter it:
```bash
mkdir build
cd build
```

Then, compile the code using CMake:
```bash
cmake ..
cmake --build . -j4
```

If everything succeeded, you can now run the following commands to interact with the dog:
```bash
./sit-down
./stand-up
```

## Structure of this Repository
The code in this repository is seperated into several directories.

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