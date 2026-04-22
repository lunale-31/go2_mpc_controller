#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <go2_utils/kinematics.h>
#include <go2_utils/robot.h>
#include <unitree_go/msg/low_cmd.hpp>

#include <stand_height/srv/stand_height.hpp>

static const std::string SERVICE_NAME = "stand_height";

using namespace std::chrono_literals;

void stand_height_request(const stand_height::srv::StandHeight::Request::SharedPtr request,
                          stand_height::srv::StandHeight::Response::SharedPtr response) {

    RCLCPP_INFO(
        rclcpp::get_logger("rclcpp"),
        "Received request for height %.4f over transition time %.4f.",
        request->height, request->transition_time);

    auto joint_configurations = go2_utils::kinematics::inverse(
        Eigen::Vector3f(0.0f, go2_utils::robot::L_1, -(request->height)),
        go2_utils::kinematics::LegSide::LEFT);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Found %ld possible joint configurations.",
                joint_configurations.size());

    for (const auto &conf : joint_configurations) {
        RCLCPP_INFO(
            rclcpp::get_logger("rclcpp"),
            " - j1: %.4f\t j2: %.4f\t j3: %.4f",
            conf.x(), conf.y(), conf.z());
    }

    response->status = joint_configurations.size();
}

/**
 * Main entry point
 * @param argc Number of program arguments
 * @param argv Program arguments
 */
int main(const int argc, char *argv[]) {

    rclcpp::init(argc, argv);

    // create controller
    const rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("stand_height_server");
    auto service = node->create_service<stand_height::srv::StandHeight>(SERVICE_NAME, stand_height_request);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Server is ready.");

    rclcpp::spin(node);

    rclcpp::shutdown();
}