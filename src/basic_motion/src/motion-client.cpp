#include <basic_motion/service.h>
#include <basic_motion/srv/damp.hpp>
#include <basic_motion/srv/gait.hpp>
#include <basic_motion/srv/stand.hpp>
#include <chrono>
#include <go2_utils/robot.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>

using namespace std::chrono_literals;

float parse_float(const char *arg) {
    float res = std::stof(arg);
    if (std::isnan(res) || std::isinf(res)) {
        throw std::invalid_argument("invalid input");
    }
    return res;
}

class RclCppContext {
public:
    RclCppContext(const int argc, char *argv[]) {
        rclcpp::init(argc, argv);
    }
    ~RclCppContext() {
        rclcpp::shutdown();
    }
};

template <typename ServiceT>
typename ServiceT::Response::SharedPtr perform_request(
    const rclcpp::Node::SharedPtr &node,
    const std::string &service_name,
    const typename ServiceT::Request::SharedPtr &request) {

    // Prepare client
    auto client = node->create_client<ServiceT>(service_name);
    if (!client->wait_for_service(100ms) || !rclcpp::ok()) {
        throw std::runtime_error("Could not establish a connection to the server.");
    }

    // Send request and await result
    auto result = client->async_send_request(request);
    switch (rclcpp::spin_until_future_complete(node, result, 100ms)) {
        case rclcpp::FutureReturnCode::INTERRUPTED:
            throw std::runtime_error("Program was interrupted before an answer was received.");
        case rclcpp::FutureReturnCode::TIMEOUT:
            throw std::runtime_error("Timeout was reached before an answer was received.");
        case rclcpp::FutureReturnCode::SUCCESS:
            return result.get();
    }
    throw std::runtime_error("Must not get here!");
}

int cmd_help([[maybe_unused]] const int argc, [[maybe_unused]] char *argv[], const rclcpp::Node::SharedPtr &node) {
    RCLCPP_INFO(node->get_logger(), "Usage: %s damp", argv[0]);
    RCLCPP_INFO(node->get_logger(), "Usage: %s stand <height> <time>", argv[0]);
    RCLCPP_INFO(node->get_logger(), "Usage: %s gait", argv[0]);
    return -1;
}

int cmd_damp([[maybe_unused]] const int argc, [[maybe_unused]] char *argv[], const rclcpp::Node::SharedPtr &node) {
    RCLCPP_INFO(node->get_logger(), "Requesting damping.");
    try {
        auto request = std::make_shared<basic_motion::srv::Damp_Request>();
        auto response = perform_request<basic_motion::srv::Damp>(node, basic_motion::SERVICE_NAME_DAMP, request);
        return response->status.code;
    } catch (std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Caught error: %s", e.what());
        return 1;
    }
}

int cmd_stand([[maybe_unused]] const int argc, [[maybe_unused]] char *argv[], [[maybe_unused]] const rclcpp::Node::SharedPtr &node) {
    try {
        auto request = std::make_shared<basic_motion::srv::Stand_Request>();
        request->height = parse_float(argv[2]);
        request->transition_time = parse_float(argv[3]);
        RCLCPP_INFO(node->get_logger(), "Requesting standing at height %.4f.", request->height);
        auto response = perform_request<basic_motion::srv::Stand>(node, basic_motion::SERVICE_NAME_STAND, request);
        return response->status.code;
    } catch (std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Caught error: %s", e.what());
        return 1;
    }
}

int cmd_gait([[maybe_unused]] const int argc, [[maybe_unused]] char *argv[], [[maybe_unused]] const rclcpp::Node::SharedPtr &node) {
    RCLCPP_INFO(node->get_logger(), "Requesting gaiting.");
    try {
        auto request = std::make_shared<basic_motion::srv::Gait_Request>();
        auto response = perform_request<basic_motion::srv::Gait>(node, basic_motion::SERVICE_NAME_GAIT, request);
        return response->status.code;
    } catch (std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Caught error: %s", e.what());
        return 1;
    }
}

/**
 * Main entry point
 * @param argc Number of program arguments
 * @param argv Program arguments
 */
int main(const int argc, char *argv[]) {
    RclCppContext ctx(argc, argv);
    const rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("basic_motion_client");

    // Load and parse arguments
    if (argc < 2) {
        return cmd_help(argc, argv, node);
    }

    std::string cmd_verb = argv[1];
    if (cmd_verb == "damp") {
        return cmd_damp(argc, argv, node);
    } else if (cmd_verb == "stand") {
        return cmd_stand(argc, argv, node);
    } else if (cmd_verb == "gait") {
        return cmd_gait(argc, argv, node);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Invalid command '%s' provided.", argv[1]);
        return cmd_help(argc, argv, node);
    }

    /*
    const float height = parse_float(argv[1]),
                transition_time = parse_float(argv[2]),
                x_offset = argc >= 4 ? parse_float(argv[3]) : 0.0f; */
}