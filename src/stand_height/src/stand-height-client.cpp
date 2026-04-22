#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <go2_utils/robot.h>
#include <unitree_go/msg/low_cmd.hpp>

#include "stand_height/srv/stand_height.hpp"

static const std::string SERVICE_NAME = "stand_height";

using namespace std::chrono_literals;

float parse_float(const char *arg) {
    try {
        float res = std::stof(arg);
        if (std::isnan(res) || std::isinf(res)) {
            throw std::invalid_argument("invalid input");
        }
        return res;
    } catch (const std::invalid_argument &) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Received the invalid argument '%s' (floating-point number expected).", arg);
        exit(1);
    }
}

/**
 * Main entry point
 * @param argc Number of program arguments
 * @param argv Program arguments
 */
int main(const int argc, char *argv[]) {
    // Load and parse arguments
    if (argc != 3) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: %s <height> <time>", argv[0]);
        return 1;
    }
    const float height = parse_float(argv[1]), transition_time = parse_float(argv[2]);

    rclcpp::init(argc, argv);

    // Prepare client
    const rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("stand_height_client");
    rclcpp::Client<stand_height::srv::StandHeight>::SharedPtr client =
        node->create_client<stand_height::srv::StandHeight>(SERVICE_NAME);
    if (!client->wait_for_service(3s) || !rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not establish a connection to the server.");
        return 1;
    }

    // Send request
    auto request = std::make_shared<stand_height::srv::StandHeight_Request>();
    request->height = height;
    request->transition_time = transition_time;
    auto result = client->async_send_request(request);

    // Await result
    switch (rclcpp::spin_until_future_complete(node, result, 1s)) {
        case rclcpp::FutureReturnCode::INTERRUPTED:
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Program was interrupted before an answer was received.");
            break;
        case rclcpp::FutureReturnCode::TIMEOUT:
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timeout was reached before an answer was received.");
            break;
        case rclcpp::FutureReturnCode::SUCCESS:
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received response %d.", result.get()->status);
            break;
    }

    rclcpp::shutdown();

    return 0;
}