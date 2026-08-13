cmake_minimum_required(VERSION 3.14)
project(controller)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Release type build helps Eigen to optimize matrices faster
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(PROJECT_WARNINGS
    -Wall
    -Wextra
  )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_go REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

find_package(std_msgs REQUIRED)
find_package(go2_interfaces REQUIRED)

# Do not compile osqp-cpp tests
set(OSQP-CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)

# Builds osqp-cpp and automatically fetches OSQP/Abseil
add_subdirectory(
  /opt/osqp-cpp
  ${CMAKE_BINARY_DIR}/osqp-cpp
  EXCLUDE_FROM_ALL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# MPC Controller
add_executable(mpc_node
  src/mpc_node.cpp
  src/mpc_node_main.cpp
  src/mpc.cpp
  src/dynamics.cpp
  src/kinematics.cpp
  src/state_estimator.cpp
  src/validation_metrics.cpp)

target_include_directories(mpc_node PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(mpc_node
  Eigen3::Eigen   
  osqp-cpp
  pinocchio::pinocchio
)

ament_target_dependencies(mpc_node
  rclcpp
  unitree_go
  go2_interfaces
  std_msgs
)

target_compile_options(mpc_node PRIVATE
  ${PROJECT_WARNINGS}
)

# State Estimator
add_executable(state_estimator_node
  src/state_estimator_node.cpp
  src/state_estimator_main.cpp
  src/dynamics.cpp
  src/kinematics.cpp
  src/state_estimator.cpp
  src/validation_metrics.cpp)

target_include_directories(state_estimator_node PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(state_estimator_node
  Eigen3::Eigen   
)

ament_target_dependencies(state_estimator_node
  rclcpp
  unitree_go
  go2_interfaces
)

target_compile_options(state_estimator_node PRIVATE
  ${PROJECT_WARNINGS}
)

# High level controller
add_executable(controller_node
  src/controller_node.cpp
  src/controller_main.cpp
  src/validation_metrics.cpp
  )

target_include_directories(controller_node PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(controller_node
  Eigen3::Eigen   
)

ament_target_dependencies(controller_node
  rclcpp
  unitree_go
  go2_interfaces
)

target_compile_options(controller_node PRIVATE
  ${PROJECT_WARNINGS}
)

# Install config and launch folder directory
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Executables 
install(TARGETS 
  state_estimator_node
  mpc_node
  controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
