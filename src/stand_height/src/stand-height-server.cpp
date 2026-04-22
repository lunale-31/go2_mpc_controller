#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <go2_utils/kinematics.h>
#include <go2_utils/robot.h>

#include "controller/Controller.h"

/**
 * Main entry point
 * @param argc Number of program arguments
 * @param argv Program arguments
 */
int main(const int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
}