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
    auto node = std::make_shared<rclcpp::Node>("stand_height_server");
    auto controller = std::make_shared<Controller>(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
}