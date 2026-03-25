//
// Created by ubuntu on 3/23/26.
//

#include <memory>
#include <chrono>
#include <string>
#include <rclcpp/node.hpp>
#include <rclcpp/executors.hpp>

#include "../interface/HighLevelControl.h"

using namespace std::chrono_literals;

/**
 * Main entry point for stand up tool
 * @param argc Number of program arguments
 * @param argv Program arguments
 */
int main(const int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // create node and executor
    const rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("stand_up");
    interface::HighLevelControl high_command(node);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // let everything get ready
    std::this_thread::sleep_for(200ms);

    // run the actual command and collect result
    auto future = high_command.stand_up();
    int result = 2;
    switch (executor.spin_until_future_complete(future, 1000ms))
    {
        case rclcpp::FutureReturnCode::SUCCESS:
        {
            const bool future_result = future.get()->header.status.code == 0;
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

    // stop and return result
    rclcpp::shutdown();
    return result;
}