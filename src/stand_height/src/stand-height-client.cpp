#include <chrono>
#include <go2_utils/robot.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stand_height/service.h>
#include <stand_height/srv/stand_height.hpp>

using namespace std::chrono_literals;

float parse_float(const char *arg) {
    try {
        float res = std::stof(arg);
        if (std::isnan(res) || std::isinf(res)) {
            throw std::invalid_argument("invalid input");
        }
        return res;
    } catch (const std::invalid_argument &) {
        RCLCPP_ERROR(rclcpp::get_logger("stand_height_client"), "Received the invalid argument '%s' (floating-point number expected).", arg);
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
        RCLCPP_ERROR(rclcpp::get_logger("stand_height_client"), "Usage: %s <height> <time>", argv[0]);
        return -1;
    }
    const float height = parse_float(argv[1]), transition_time = parse_float(argv[2]);

    rclcpp::init(argc, argv);

    // Prepare client
    const rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("stand_height_client");
    auto client = node->create_client<stand_height::srv::StandHeight>(stand_height::SERVICE_NAME);
    if (!client->wait_for_service(100ms) || !rclcpp::ok()) {
        RCLCPP_ERROR(node->get_logger(), "Could not establish a connection to the server.");
        return 1;
    }

    // Prepare request
    auto request = std::make_shared<stand_height::srv::StandHeight_Request>();
    request->height = height;
    request->transition_time = transition_time;
    
    // Send request and await result
    int return_code = -2;
    auto result = client->async_send_request(request);
    switch (rclcpp::spin_until_future_complete(node, result, 100ms)) {
        case rclcpp::FutureReturnCode::INTERRUPTED:
            RCLCPP_ERROR(node->get_logger(), "Program was interrupted before an answer was received.");
            return_code = 2;
            break;
        case rclcpp::FutureReturnCode::TIMEOUT:
            RCLCPP_ERROR(node->get_logger(), "Timeout was reached before an answer was received.");
            return_code = 3;
            break;
        case rclcpp::FutureReturnCode::SUCCESS:
            RCLCPP_INFO(node->get_logger(), "Received response %d.", result.get()->status);
            return_code = 0;
            break;
    }

    rclcpp::shutdown();

    return return_code;
}