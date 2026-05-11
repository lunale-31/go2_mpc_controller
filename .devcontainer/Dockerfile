FROM ros:humble

SHELL ["bash", "-lc"]

RUN apt-get update && \
    apt-get upgrade -y --autoremove && \
    apt-get install -y build-essential cmake git curl iproute2 gdb python3-ipykernel libyaml-cpp-dev python3-pandas openssh-client \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp ros-${ROS_DISTRO}-rosidl-generator-dds-idl ros-${ROS_DISTRO}-ament-cmake-clang-format

RUN rosdep update

# Install Unitree Go2 ROS2 bindings
WORKDIR /tmp
RUN git clone https://github.com/unitreerobotics/unitree_ros2.git -b v0.3.0
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && cd unitree_ros2 && colcon build --install-base /opt/unitree_ros2

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
       groupadd --gid $USER_GID $USERNAME && \
       useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi

# Allow user to use sudo
RUN echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && chmod 0440 /etc/sudoers.d/$USERNAME

## Switch from root to user
USER $USERNAME

# Immediately update bash history
RUN echo "shopt -s histappend" >> ~/.bashrc
RUN echo "PROMPT_COMMAND+=(\"history -a\")" >> ~/.bashrc

# Setup Unitree parameters
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /opt/unitree_ros2/setup.bash" >> ~/.bashrc
RUN echo "if [ -d /workspace/install ]; then source /workspace/install/setup.bash; else echo Initialize the workspace by running 'colcon build'.; fi" >> ~/.bashrc

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV UNITREE_ROS2_CONTAINER=true