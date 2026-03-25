cmake_minimum_required(VERSION 3.22)
project(rtsc_unitree_ros2)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++23
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 23)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()


include_directories(include include/common include/nlohmann)
link_directories(src)

set(
        DEPENDENCY_LIST
        unitree_go
        unitree_hg
        unitree_api
        rclcpp
        std_msgs
        rosbag2_cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# define executables
add_executable(
        highlevel
        src/interface/HighLevelControl.cpp
        src/main.cpp
)

add_executable(
        stand-up
        src/interface/HighLevelControl.cpp
        src/entrypoints/stand-up.cpp
)

add_executable(
        sit-down
        src/interface/HighLevelControl.cpp
        src/entrypoints/sit-down.cpp
)

add_executable(
        lowlevel
        src/nodes/StandHeightController.cpp
        src/entrypoints/lowlevel.cpp
        src/interface/lowlevel/Motor.cpp
        src/interface/lowlevel/Leg.cpp
        src/interface/LowLevelControl.cpp
)

# define IDL interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
        "interfaces/srv/StandHeight.srv"
)

ament_target_dependencies(highlevel ${DEPENDENCY_LIST})
ament_target_dependencies(stand-up ${DEPENDENCY_LIST})
ament_target_dependencies(sit-down ${DEPENDENCY_LIST})
ament_target_dependencies(lowlevel ${DEPENDENCY_LIST})

install(
        TARGETS
        highlevel stand-up
        ARCHIVE DESTINATION "lib"
        LIBRARY DESTINATION "lib"
        RUNTIME DESTINATION "bin"
        PUBLIC_HEADER DESTINATION "include"
)

ament_package()