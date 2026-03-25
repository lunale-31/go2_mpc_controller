//
// Created by ubuntu on 3/23/26.
//

#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/executors.hpp>

#include "../nodes/StandHeightController.h"

/**
 * Main entry point
 * @param argc Number of program arguments
 * @param argv Program arguments
 */
int main(const int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // create nodes
    const rclcpp::Node::SharedPtr standHeightNode = std::make_shared<nodes::StandHeightController>();

    // execute
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(standHeightNode);
    executor.spin();
    rclcpp::shutdown();
}