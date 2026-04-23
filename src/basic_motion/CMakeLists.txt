cmake_minimum_required(VERSION 3.8)
project(basic_motion)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++23
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 23)
endif ()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(go2_utils REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# generate IDL headers
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Status.msg"
  "srv/Damp.srv"
  "srv/Gait.srv"
  "srv/Stand.srv"
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

include_directories(include)

set(TARGETS motion-server motion-client)

# Stand height server
add_executable(
  motion-server
  src/motion-server.cpp
  src/controller/Controller.cpp
  src/controller/JointPose.cpp
  src/controller/JointPoseInterpolation.cpp
)

# Stand height client
add_executable(
  motion-client
  src/motion-client.cpp
)

foreach(TARGET IN LISTS TARGETS)
  ament_target_dependencies(${TARGET} go2_utils rclcpp)
  target_link_libraries(${TARGET} "${cpp_typesupport_target}")
endforeach()

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
ament_export_include_directories(include)
install(
  DIRECTORY "include/${PROJECT_NAME}"
  DESTINATION include/
)

# install everything
install(
  TARGETS ${TARGETS}
  EXPORT "${PROJECT_NAME}"
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin"
  INCLUDES DESTINATION "include")
install(
  TARGETS ${TARGETS}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
