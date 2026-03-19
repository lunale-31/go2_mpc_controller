# Unitree Go2 Dog Control

This repo contains code used for controlling the Unitree Go2 robot dog using ROS2.

## Development Containers
The code found in this repository is assumed to be executed inside a development container. 
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

TODO: Continue.
```
ros2 topic list
```