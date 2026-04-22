cmake_minimum_required(VERSION 3.8)
project(stand_height)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(go2_utils REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# generate IDL headers
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StandHeight.srv"
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

include_directories(include)

# Stand height server
add_executable(
  stand-height-server
  src/stand-height-server.cpp
  src/controller/Controller.cpp
)
ament_target_dependencies(stand-height-server
  go2_utils
  rclcpp
)
target_link_libraries(stand-height-server "${cpp_typesupport_target}")


# Stand height client
add_executable(
  stand-height-client
  src/stand-height-client.cpp
  src/controller/Controller.cpp
)
ament_target_dependencies(stand-height-client
  go2_utils
  rclcpp
)
target_link_libraries(stand-height-client "${cpp_typesupport_target}")


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

# install public headers
install(
  DIRECTORY "include/${PROJECT_NAME}"
  DESTINATION include/
)
ament_export_include_directories(include)

# install everything
install(
  TARGETS stand-height-client stand-height-server
  EXPORT "${PROJECT_NAME}"
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin"
  INCLUDES DESTINATION "include"
)
install(TARGETS
  stand-height-client stand-height-server
  DESTINATION lib/${PROJECT_NAME})

ament_package()
