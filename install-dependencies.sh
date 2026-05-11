#!/bin/sh
WORKDIR=$(pwd)
REPODIR=$(dirname "$0")
TMPDIR=$(mktmp -d)

# Make sure we are not running as root
if [ "$(whoami)" = "root" ]; then
    echo "Do not run this script as root."
    exit 1
fi

# Make sure we are not inside the development container
if [ -n "$UNITREE_ROS2_CONTAINER" ]; then
    echo "Do not run this script within the development container."
    exit 1
fi

# Get, build and install Unitree's ROS2 library
git clone https://github.com/unitreerobotics/unitree_ros2.git -b v0.3.0 "$TMPDIR"
cd "$TMPDIR"
source /opt/ros/${ROS_DISTRO}/setup.bash
sudo colcon build --install-base /opt/unitree_ros2
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
echo 'export PS1="\${dds_prompt:+[DDS: \${dds_prompt}] }\$PS1"' >> ~/.bashrc
rm -rf "$TMPDIR"

# Build the code
cd "$REPODIR"
colcon build

# Switch back to the working directory
cd "$WORKDIR"