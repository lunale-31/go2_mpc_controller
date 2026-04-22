#include "Controller.h"
#include <functional>
#include <go2_utils/kinematics.h>
#include <go2_utils/robot.h>
#include <stand_height/service.h>


Controller::Controller() : Node("stand_height_server") {
    using namespace std::placeholders;
    using namespace std::chrono_literals;

    service_ = create_service<stand_height::srv::StandHeight>(
        stand_height::SERVICE_NAME, 
        std::bind(&Controller::service_request, this, _1, _2));
    
    timer_ = create_wall_timer(2ms, std::bind(&Controller::timer_tick, this));

    RCLCPP_INFO(get_logger(), "Server is ready.");
}

void Controller::timer_tick() {

}

void Controller::service_request(
    const stand_height::srv::StandHeight::Request::SharedPtr request,
    stand_height::srv::StandHeight::Response::SharedPtr response) {
    RCLCPP_INFO(
        get_logger(),
        "Received request for height %.4f over transition time %.4f.",
        request->height, request->transition_time);

    auto joint_configurations = go2_utils::kinematics::inverse(
        Eigen::Vector3f(0.0f, go2_utils::robot::L_1, -(request->height)),
        go2_utils::kinematics::LegSide::LEFT);

    RCLCPP_INFO(get_logger(),
                "Found %ld possible joint configurations.",
                joint_configurations.size());

    for (const auto &conf : joint_configurations) {
        RCLCPP_INFO(
            get_logger(),
            " - j1: %.4f\t j2: %.4f\t j3: %.4f",
            conf.x(), conf.y(), conf.z());
    }

    response->status = joint_configurations.size();
}
