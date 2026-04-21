#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <common/go_constants.h>
#include <unitree_go/msg/low_cmd.hpp>

#include "../common/Kinematics.h"
#include "rtsc_unitree_ros2/srv/stand_height.hpp"

static const std::string SERVICE_NAME = "stand_height";

using namespace std::chrono_literals;

void stand_height_request(const rtsc_unitree_ros2::srv::StandHeight::Request::SharedPtr request,
                          rtsc_unitree_ros2::srv::StandHeight::Response::SharedPtr response) {

    RCLCPP_INFO(
        rclcpp::get_logger("rclcpp"),
        "Received request for height %.4f over transition time %.4f.",
        request->height, request->transition_time);

    auto joint_configurations = common::inverse_kinematics(
        Eigen::Vector3f(0.0f, common::L_1, -(request->height)),
        common::LegSide::LEFT);

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
    node->create_service<rtsc_unitree_ros2::srv::StandHeight>(SERVICE_NAME, stand_height_request);

    rclcpp::shutdown();
}