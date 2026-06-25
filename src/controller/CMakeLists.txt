cmake_minimum_required(VERSION 3.8)
project(controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_go REQUIRED)

find_package(Eigen3 REQUIRED)

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

# Dynamics test
add_executable(dynamics_test
  src/go2_mpc.cpp
  src/main.cpp
  src/dynamics.cpp
  src/kinematics.cpp)

target_link_libraries(dynamics_test
  Eigen3::Eigen   
)

ament_target_dependencies(dynamics_test
  rclcpp
  unitree_go
)

# Install config and launch folder directory
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Executables 
install(TARGETS 
  dynamics_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
