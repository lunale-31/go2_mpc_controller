# Getting Started

This document contains information about the robot and the software in this repository.
Please read it carefully before running any code.

## Official Documentation

Unitree provides [official documentation](https://support.unitree.com/home/en/developer) for its Go2 series of robots.
The code in this repository is built around the (outdated) [ROS2 Humble](https://docs.ros.org/en/humble/index.html) and requires Ubuntu 22.04 (which is the newest version compatible with the robot).

## Interfacing with the Robot

To interface with the robot, you must connect to the robot via ethernet.
You need to configure your ethernet interface to the following IPv4 settings:
 - IP address: `192.168.123.222`
 - Network mask: `255.255.255.0` or `24`

We can interface with the robot via the DDS middleware.
The middleware is configured using environment variables.
You can run one of the following commands to configure the environment:

```sh
# Configures DDS to communicate with the robot via ethernet.
source dds-robot

# Configures DDS to communicate locally, e.g., with a simulation.
source dds-loopback
```

You need to correctly set the environment before running any programs in this repository.

## Enabling the ROS2 Environment

The code in this repository is a collection of ROS2 packages.
To run it, you first need to activate them by sourcing the setup script:
```sh
source install/setup.bash 
```

If this script does not exist, you first need to build the code using `colcon`:
```sh
colcon build
```

## High- and Low-level Control

Unitree's Go2 robots can be operated in two modes: Using the built-in _high-level_ controller, or using an external _low-level_ controller.
The high-level controller is enabled by default and allows to control the robot using commands like _walk_, _stand up_, _jump_, or _greet_ (see [Unitree's documentation](https://support.unitree.com/home/en/developer/Motion_Services_Interface_V2.0)).
During high-level control, it is not possible to manually control joint torques as they get overridden by values from the built-in controller.

> [!caution]
> Sending torque commands to the robot while the high-level controller is active can lead to rapid, unpredictable motions.

To control the robot with _low-level_ torque commands, the built-in controller must be deactivated.
We provide a tool for switching the built-in controller on or off using the [Motion Switcher Service Interface](https://support.unitree.com/home/en/developer/Motion%20Switcher%20Service%20Interface):
```sh
# Turn the high-level controller off
ros2 run go2_cli motion-switcher off
```

After running the command, you need to turn the robot off and back on.
If the high-level controller was successfully deactivated, the robot will no longer stand up after being turned on.
Only once that is the case, you are ready to start running your own controller.

To reenable the high-level controller, run:
```sh
# Turn the high-level controller on
ros2 run go2_cli motion-switcher on
```

Again, you need to turn the robot off and back on for the change to take effect.