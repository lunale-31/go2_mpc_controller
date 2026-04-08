#include <chrono>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>

#include "../controllers/standheight/Controller.h"

#include <common/go_constants.h>
#include <unitree_go/msg/low_cmd.hpp>

using namespace std::chrono_literals;

/**
 * Main entry point
 * @param argc Number of program arguments
 * @param argv Program arguments
 */
int main(const int argc, char *argv[]) {
    // Load and parse config
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <config_file>" << std::endl;
        return -1;
    }
    auto config = controllers::standheight::Config::load(argv[1]);

    rclcpp::init(argc, argv);

    // create controller
    const rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("stand_height_node");
    controllers::standheight::Controller controller(node, config);

    std::this_thread::sleep_for(200ms);

    // execute
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
}