#include "discontinuous/LegController.h"
#include <go2_utils/interact/LowLevelControl.h>
#include <rclcpp/rclcpp.hpp>

using namespace go2_utils::interact;

void await_data(const rclcpp::Node::SharedPtr &node, const std::shared_ptr<LowLevelControl> &llc) {
    RCLCPP_INFO(node->get_logger(), "Waiting for a response from the robot.");
    while (!llc->was_state_received()) {
        rclcpp::spin_some(node);
    }
    RCLCPP_INFO(node->get_logger(), "Received a response from the robot.");
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    const auto node = std::make_shared<rclcpp::Node>("gait_logger");
    const auto llc = std::make_shared<LowLevelControl>(node);

    await_data(node, llc);

    for (int i = 0; i < 12; ++i) {
        auto &joint = llc->joint(i);
        joint->cmd().kp = 80;
        joint->cmd().kd = 1;
    }

    LegController front_left(llc->frontLeft(), 4, LegController::MIDDLE, LegController::MIDDLE);
    LegController front_right(llc->frontRight(), 1, LegController::BACKWARDS, LegController::BACKWARDS);
    LegController back_left(llc->backLeft(), 3, LegController::MIDDLE, LegController::MIDDLE);
    LegController back_right(llc->backRight(), 0, LegController::BACKWARDS, LegController::FORWARDS);

    const auto timer = node->create_wall_timer(std::chrono::milliseconds(2), [&] {
        front_left.tick();
        front_right.tick();
        back_left.tick();
        back_right.tick();
        llc->publish();
    });

    rclcpp::spin(node);

    rclcpp::shutdown();
}