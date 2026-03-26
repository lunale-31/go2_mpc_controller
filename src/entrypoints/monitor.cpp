//
// Created by ubuntu on 3/23/26.
//

#include <memory>
#include <chrono>
#include <rclcpp/node.hpp>
#include <rclcpp/executors.hpp>
#include <unitree_go/msg/low_cmd.hpp>
#include <unitree_api/msg/request.hpp>
#include <unitree_api/msg/response.hpp>
#include <common/go_constants.h>

#include "../interface/HighLevelControl.h"

using namespace std::chrono_literals;

/**
 * Main entry point for sit down tool
 * @param argc Number of program arguments
 * @param argv Program arguments
 */
int main(const int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // create node and subscribe to /lowcmd
    const rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("monitor");
    interface::HighLevelControl high_command(node);
    std::this_thread::sleep_for(200ms);
    // /*
    auto subscription = node->create_subscription<unitree_go::msg::LowCmd>(
        "/lowcmd", 10, [node](const unitree_go::msg::LowCmd &msg)
        { 
            if (msg.motor_cmd[common::constants::BL_HIP].q == 0.0f) return;

            RCLCPP_INFO(node->get_logger(),
                      "BL Hip: q = %f, dq = %f, tau = %f, kp = %f, kd = %f | "
                      "BL Thigh: q = %f, dq = %f, tau = %f, kp = %f, kd = %f | "
                      "BL Calf: q = %f, dq = %f, tau = %f, kp = %f, kd = %f",
                      msg.motor_cmd[common::constants::BL_HIP].q,
                      msg.motor_cmd[common::constants::BL_HIP].dq,
                      msg.motor_cmd[common::constants::BL_HIP].tau,
                      msg.motor_cmd[common::constants::BL_HIP].kp,
                      msg.motor_cmd[common::constants::BL_HIP].kd,
                      msg.motor_cmd[common::constants::BL_THIGH].q,
                      msg.motor_cmd[common::constants::BL_THIGH].dq,
                      msg.motor_cmd[common::constants::BL_THIGH].tau,
                      msg.motor_cmd[common::constants::BL_THIGH].kp,
                      msg.motor_cmd[common::constants::BL_THIGH].kd,
                      msg.motor_cmd[common::constants::BL_CALF].q,
                      msg.motor_cmd[common::constants::BL_CALF].dq,
                      msg.motor_cmd[common::constants::BL_CALF].tau,
                      msg.motor_cmd[common::constants::BL_CALF].kp,
                      msg.motor_cmd[common::constants::BL_CALF].kd); });

    // */
    auto ms_req_subs = node->create_subscription<unitree_api::msg::Request>(
        "/api/motion_switcher/request", 10, [node](const unitree_api::msg::Request &msg) {
            RCLCPP_INFO(node->get_logger(), "MS Request:\n%s", to_yaml(msg).c_str());
        });
    auto ms_res_subs = node->create_subscription<unitree_api::msg::Response>(
        "/api/motion_switcher/response", 10, [node](const unitree_api::msg::Response &msg) {
            RCLCPP_INFO(node->get_logger(), "MS Response:\n%s", to_yaml(msg).c_str());
        });

    // execute node
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // run stand up command
    auto future = high_command.sit_down();
    int result = 2;
    switch (executor.spin_until_future_complete(future, 1000ms))
    {
    case rclcpp::FutureReturnCode::SUCCESS:
    {
        const bool future_result = future.get();
        result = future_result ? 0 : 1;
        RCLCPP_INFO(node->get_logger(), "Received result %s", future_result ? "SUCCESS" : "FAILURE");
        break;
    }
    case rclcpp::FutureReturnCode::TIMEOUT:
        RCLCPP_ERROR(node->get_logger(), "Ran into timeout.");
        break;
    case rclcpp::FutureReturnCode::INTERRUPTED:
        RCLCPP_ERROR(node->get_logger(), "Node got interrupted.");
        break;
    }

    executor.spin();

    // stop and return result
    rclcpp::shutdown();
    return result;
}