//
// Created by ubuntu on 3/23/26.
//

#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/executors.hpp>


/**
 * Main entry point
 * @param argc Number of program arguments
 * @param argv Program arguments
 */
int main(const int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    // nothing
    rclcpp::shutdown();
}