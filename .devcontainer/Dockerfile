FROM ros:humble

SHELL ["bash", "-lc"]

RUN apt-get update && \
    apt-get upgrade -y --autoremove && \
    apt-get install -y build-essential cmake openssh-client git curl iproute2 gdb libyaml-cpp-dev libeigen3-dev python3-ipykernel python3-pandas python3-ipywidgets \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp ros-${ROS_DISTRO}-rosidl-generator-dds-idl ros-${ROS_DISTRO}-ament-cmake-clang-format \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp ros-${ROS_DISTRO}-rosidl-generator-dds-idl ros-${ROS_DISTRO}-ament-cmake-clang-format \
        ros-${ROS_DISTRO}-foxglove-bridge ros-${ROS_DISTRO}-plotjuggler-ros \ 
        ros-${ROS_DISTRO}-rosbag2-storage-mcap \
        ros-${ROS_DISTRO}-pinocchio
RUN pip install ipympl
RUN rosdep update

# Install OSQP solver
# Google C++ wrapper
RUN git clone --depth 1 \
    https://github.com/google/osqp-cpp.git \
    /opt/osqp-cpp

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
RUN echo 'export PS1="\${dds_prompt:+\033[37m[DDS \$dds_prompt]\033[0m }$PS1"' >> ~/.bashrc


ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV UNITREE_ROS2_CONTAINER=true