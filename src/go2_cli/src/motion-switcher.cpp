#include <chrono>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <string>
#include <go2_utils/interface/MotionSwitcher.h>

using namespace std::chrono_literals;

/**
 * Main entry point for stand up tool
 * @param argc Number of program arguments
 * @param argv Program arguments
 */
int main(const int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <on|off>" << std::endl;
        return -1;
    }

    // parse argument
    bool silent;
    std::string command = argv[1];
    if (command == "on") {
        // enable motion
        silent = false; // silent is set inversely to on and off
    } else if (command == "off") {
        // disable motion
        silent = true; // silent is set inversely to on and off
    } else {
        std::cerr << "Usage: " << argv[0] << " <on|off>" << std::endl;
        return -2;
    }

    // initialize rclcpp
    rclcpp::init(argc, argv);

    // create node and executor
    const rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("motion_switcher");
    go2_utils::interface::MotionSwitcher motion_switcher(node);
    rclcpp::spin_some(node);

    // let everything get ready
    std::this_thread::sleep_for(200ms);

    // run the actual command and collect result
    RCLCPP_INFO(node->get_logger(), "%s silent mode.", silent ? "Enabling" : "Disabling");
    auto future = motion_switcher.set_silent(silent);
    int result = 1;
    switch (rclcpp::spin_until_future_complete(node, future, 1000ms)) {
        case rclcpp::FutureReturnCode::SUCCESS: {
            const bool future_result = future.get();
            result = future_result ? 0 : 1;
            RCLCPP_INFO(node->get_logger(), "Received result %s", future_result ? "SUCCESS" : "FAILURE");
            break;
        }
        case rclcpp::FutureReturnCode::TIMEOUT:
            RCLCPP_ERROR(node->get_logger(), "Ran into timeout.");
            result = 2;
            break;
        case rclcpp::FutureReturnCode::INTERRUPTED:
            RCLCPP_ERROR(node->get_logger(), "Node got interrupted.");
            result = 3;
            break;
    }

    // run check command
    if (result == 0) {
        RCLCPP_INFO(node->get_logger(), "Checking silent mode.");
        auto future = motion_switcher.get_silent();
        switch (rclcpp::spin_until_future_complete(node, future, 1000ms)) {
            case rclcpp::FutureReturnCode::SUCCESS: {
                const bool future_result = future.get();
                result = future_result == silent ? 0 : 1;
                RCLCPP_INFO(node->get_logger(), "Check %s.", (future_result == silent) ? "succeeded" : "failed");
                break;
            }
            case rclcpp::FutureReturnCode::TIMEOUT:
                RCLCPP_ERROR(node->get_logger(), "Ran into timeout.");
                result = 2;
                break;
            case rclcpp::FutureReturnCode::INTERRUPTED:
                RCLCPP_ERROR(node->get_logger(), "Node got interrupted.");
                result = 3;
                break;
        }
    }

    // stop and return result
    rclcpp::shutdown();
    return result;
}