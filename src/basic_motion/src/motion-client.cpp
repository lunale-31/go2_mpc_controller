#include <chrono>
#include <go2_utils/robot.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <basic_motion/service.h>
#include <basic_motion/srv/stand.hpp>

using namespace std::chrono_literals;

float parse_float(const char *arg) {
    try {
        float res = std::stof(arg);
        if (std::isnan(res) || std::isinf(res)) {
            throw std::invalid_argument("invalid input");
        }
        return res;
    } catch (const std::invalid_argument &) {
        RCLCPP_ERROR(rclcpp::get_logger("basic_motion_client"), "Received the invalid argument '%s' (floating-point number expected).", arg);
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
    if (argc < 3) {
        RCLCPP_ERROR(rclcpp::get_logger("basic_motion_client"), "Usage: %s <height> <time> [x_offset]", argv[0]);
        return -1;
    }
    const float height = parse_float(argv[1]),
                transition_time = parse_float(argv[2]),
                x_offset = argc >= 4 ? parse_float(argv[3]) : 0.0f;

    rclcpp::init(argc, argv);

    // Prepare client
    const rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("basic_motion_client");
    auto client = node->create_client<basic_motion::srv::Stand>(basic_motion::SERVICE_NAME_STAND);
    if (!client->wait_for_service(100ms) || !rclcpp::ok()) {
        RCLCPP_ERROR(node->get_logger(), "Could not establish a connection to the server.");
        return 1;
    }

    // Prepare request
    auto request = std::make_shared<basic_motion::srv::Stand_Request>();
    request->height = height;
    request->transition_time = transition_time;
    request->x_offset = x_offset;

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
            RCLCPP_INFO(node->get_logger(), "Received response %d.", result.get()->status.code);
            return_code = 0;
            break;
    }

    rclcpp::shutdown();

    return return_code;
}