#!/bin/bash
set -e

WORKDIR="$(dirname $0)"
LIBDIR="$WORKDIR/lib"
pushd $WORKDIR

mkdir -p "$LIBDIR"

if [ -d "$LIBDIR/unitree" ]; then
    echo "Unitree ROS2 library found."
else
    echo "Fetching Unitree ROS2 library..."
    TMPDIR="$(mktemp -d)"
    git clone "https://github.com/unitreerobotics/unitree_ros2" "$TMPDIR"
    mv "$TMPDIR/cyclonedds_ws/src/unitree" "$LIBDIR/unitree"
    rm -rf "$TMPDIR/"
fi

if [ -d "$LIBDIR/cyclonedds" ]; then
    echo "CycloneDDS library found."
else
    echo "Fetching CycloneDDS..."
    git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.9.x "$LIBDIR/cyclonedds"
fi

if [ -d "$LIBDIR/rmw_cyclonedds" ]; then
    echo "CycloneDDS RMW library found."
else
    echo "Fetching CycloneDDS RMW..."
    git clone https://github.com/ros2/rmw_cyclonedds -b ${ROS_DISTRO} "$LIBDIR/rmw_cyclonedds"
fi

echo "Building CycloneDDS..."
unset LD_LIBRARY_PATH
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --packages-select cyclonedds rmw_cyclonedds_cpp unitree_api unitree_go unitree_hg
source install/setup.bash
echo "Successfully installed dependencies!"

popd