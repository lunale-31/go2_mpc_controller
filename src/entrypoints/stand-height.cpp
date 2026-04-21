#include <chrono>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>

#include "../controllers/stand-height/Controller.h"

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
    auto config = controllers::stand_height::Config::load(argv[1]);

    rclcpp::init(argc, argv);

    // create controller
    const rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("stand_height");
    controllers::stand_height::Controller controller(node, config);

    std::this_thread::sleep_for(200ms);

    // execute
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin_until_future_complete(controller.done_future());

    // unlock all motors
    auto &llc = controller.low_level_control();
    for (int i = 0; i < 12; i++) {
        llc->joint(i)->mode(0);
    }
    llc->publish();
    executor.spin_some(1ms);

    rclcpp::shutdown();
}