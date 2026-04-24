#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <go2_utils/kinematics.h>
#include <go2_utils/robot.h>

#include "controller/MotionController.h"

/**
 * Main entry point
 * @param argc Number of program arguments
 * @param argv Program arguments
 */
int main(const int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("stand_height_server");
    basic_motion::controller::MotionController controller(node, 0.002f /* seconds */);
    rclcpp::spin_until_future_complete(node, controller.termination_future());
    rclcpp::shutdown();
}