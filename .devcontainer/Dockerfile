FROM ros:humble

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
       groupadd --gid $USER_GID $USERNAME && \
       useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi

# Override default shell
SHELL ["/bin/bash", "-c"]

# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo apt-utils ssh && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

## Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt-get update && sudo apt-get upgrade -y

# Install Git and CycloneDDS dependencies
RUN sudo apt-get install -y git build-essential libyaml-cpp-dev ros-${ROS_DISTRO}-rmw-cyclonedds-cpp ros-${ROS_DISTRO}-rosidl-generator-dds-idl ros-${ROS_DISTRO}-ament-cmake-clang-format iproute2 python3-ipykernel python3-pandas

# Rosdep update
RUN rosdep update

# Clone and compile cyclonedds
RUN git clone https://github.com/unitreerobotics/unitree_ros2 ~/unitree_ros2
RUN git clone https://github.com/ros2/rmw_cyclonedds -b ${ROS_DISTRO} ~/unitree_ros2/cyclonedds_ws/src/rmw_cyclonedds
RUN git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x ~/unitree_ros2/cyclonedds_ws/src/cyclonedds
WORKDIR /home/$USERNAME/unitree_ros2/cyclonedds_ws
RUN colcon build --packages-select cyclonedds
RUN source /opt/ros/${ROS_DISTRO}/setup.bash; colcon build

# Setup Unitree parameters
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source ~/unitree_ros2/cyclonedds_ws/install/setup.bash" >> ~/.bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.profile
RUN echo "source ~/unitree_ros2/cyclonedds_ws/install/setup.bash" >> ~/.profile
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>true</AllowMulticast><Interfaces><NetworkInterface address="192.168.123.0" /></Interfaces></General></Domain></CycloneDDS>'

# Uncomment the following lines if the network interface is not selected automatically.
# If necessary, change the network interface name from eth0.
# ARG NETWORK_IF=eth0
# ENV CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="$NETWORK_IF" priority="default" multicast="default" /></Interfaces><AllowMulticast>spdp</AllowMulticast></General></Domain></CycloneDDS>'
