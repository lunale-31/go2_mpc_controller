cmake_minimum_required(VERSION 3.8)
project(go2_cli)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(go2_utils REQUIRED)

add_executable(hl-sit-down
    src/hl-sit-down.cpp
)
ament_target_dependencies(hl-sit-down
    go2_utils
    rclcpp
)

add_executable(hl-stand-up
    src/hl-stand-up.cpp
)
ament_target_dependencies(hl-stand-up
    go2_utils
    rclcpp
)

add_executable(motion-switcher
    src/motion-switcher.cpp
)
ament_target_dependencies(motion-switcher
    go2_utils
    rclcpp
)

install(
    TARGETS hl-sit-down hl-stand-up motion-switcher
    ARCHIVE DESTINATION "lib"
    LIBRARY DESTINATION "lib"
    RUNTIME DESTINATION "bin"
    PUBLIC_HEADER DESTINATION "include"
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

ament_package()
