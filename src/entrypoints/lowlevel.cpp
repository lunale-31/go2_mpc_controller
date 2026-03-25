//
// Created by ubuntu on 3/23/26.
//

#include <memory>
#include <chrono>
#include <rclcpp/node.hpp>
#include <rclcpp/executors.hpp>

#include "../controllers/StandHeightController.h"
#include "../interface/HighLevelControl.h"

#include <unitree_go/msg/low_cmd.hpp>
#include <common/go_constants.h>


using namespace std::chrono_literals;

/**
 * Main entry point
 * @param argc Number of program arguments
 * @param argv Program arguments
 */
int main(const int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // create controller
    const rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("stand_height_node");
    controllers::StandHeightController controller(node);
    interface::HighLevelControl high_command(node);

    // disable high-level controller
    std::this_thread::sleep_for(200ms);
    high_command.stop_move();

    const auto subscription = node->create_subscription<unitree_go::msg::LowCmd>(
        "/lowcmd", 10, [node](const unitree_go::msg::LowCmd &msg)
        { 
            // if (msg.motor_cmd[common::constants::BR_HIP].q == 0.0f) return;

            RCLCPP_INFO(node->get_logger(),
                      "BR Hip: q = %f, dq = %f, tau = %f, kp = %f, kd = %f | "
                      "BR Thigh: q = %f, dq = %f, tau = %f, kp = %f, kd = %f | "
                      "BR Calf: q = %f, dq = %f, tau = %f, kp = %f, kd = %f",
                      msg.motor_cmd[common::constants::BR_HIP].q,
                      msg.motor_cmd[common::constants::BR_HIP].dq,
                      msg.motor_cmd[common::constants::BR_HIP].tau,
                      msg.motor_cmd[common::constants::BR_HIP].kp,
                      msg.motor_cmd[common::constants::BR_HIP].kd,
                      msg.motor_cmd[common::constants::BR_THIGH].q,
                      msg.motor_cmd[common::constants::BR_THIGH].dq,
                      msg.motor_cmd[common::constants::BR_THIGH].tau,
                      msg.motor_cmd[common::constants::BR_THIGH].kp,
                      msg.motor_cmd[common::constants::BR_THIGH].kd,
                      msg.motor_cmd[common::constants::BR_CALF].q,
                      msg.motor_cmd[common::constants::BR_CALF].dq,
                      msg.motor_cmd[common::constants::BR_CALF].tau,
                      msg.motor_cmd[common::constants::BR_CALF].kp,
                      msg.motor_cmd[common::constants::BR_CALF].kd); });

    // execute
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
}