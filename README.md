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
Remember or write that name down for troubleshooting.

Now, you can query the list of ROS 2 topics provided by the robot dog. 
Run the following command.
```bash
ros2 topic list
```
If everything is working as intended, you will receive a long list of ROS 2 topics:
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
