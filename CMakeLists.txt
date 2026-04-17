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
        Eigen3
        yaml-cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# define library
add_library(
        go2interface
        SHARED
        src/common/Kinematics.cpp
        src/common/PidController.cpp
        src/interface/HighLevelControl.cpp
        src/interface/highlevel/UnitreeApi.cpp
        src/interface/LowLevelControl.cpp
        src/interface/lowlevel/Joint.cpp
        src/interface/lowlevel/Leg.cpp
        src/interface/MotionSwitcher.cpp
)

# define executables
add_executable(
        stand-up
        src/entrypoints/stand-up.cpp
)

add_executable(
        sit-down
        src/entrypoints/sit-down.cpp
)

add_executable(
        lowlevel
        src/entrypoints/lowlevel.cpp
        src/controllers/standheight/Controller.cpp
        src/controllers/standheight/states/LegBalancer.cpp
        src/controllers/standheight/states/MotionSwitcherCheck.cpp
        src/controllers/standheight/states/MotionSwitcherSet.cpp
        src/controllers/standheight/controller/CascadedJointController.cpp
)

add_executable(
        square-wave
        src/entrypoints/square-wave.cpp
)

add_executable(
        sine-wave
        src/entrypoints/sine-wave.cpp
)

add_executable(
        cascaded-sine-wave
        src/entrypoints/cascaded-sine-wave.cpp
)

add_executable(
        motion-switcher
        src/entrypoints/motion-switcher.cpp
)

add_executable(
        kinematics-test
        src/entrypoints/kinematics-test.cpp
)

add_executable(
        command-recorder
        src/entrypoints/command-recorder.cpp
)

add_executable(
        pose-recorder
        src/entrypoints/pose-recorder.cpp
)


# define IDL interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
        "interfaces/srv/StandHeight.srv"
)

ament_target_dependencies(go2interface ${DEPENDENCY_LIST})

target_link_libraries(lowlevel PRIVATE go2interface)
target_link_libraries(stand-up PRIVATE go2interface)
target_link_libraries(sit-down PRIVATE go2interface)
target_link_libraries(square-wave PRIVATE go2interface)
target_link_libraries(sine-wave PRIVATE go2interface)
target_link_libraries(cascaded-sine-wave PRIVATE go2interface)
target_link_libraries(motion-switcher PRIVATE go2interface)
target_link_libraries(kinematics-test PRIVATE go2interface)
target_link_libraries(command-recorder PRIVATE go2interface)
target_link_libraries(pose-recorder PRIVATE go2interface)

install(
        TARGETS
        stand-up sit-down
        ARCHIVE DESTINATION "lib"
        LIBRARY DESTINATION "lib"
        RUNTIME DESTINATION "bin"
        PUBLIC_HEADER DESTINATION "include"
)

ament_package()