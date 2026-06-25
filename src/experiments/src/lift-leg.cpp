#include <go2_utils/interact/LowLevelControl.h>
#include <go2_utils/robot.h>
#include <go2_utils/kinematics.h>
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

    const auto foot_down_angles_fl = go2_utils::kinematics::inverse(Eigen::Vector3f(
        0.01f, go2_utils::robot::L_1, -0.20f
    ), go2_utils::robot::LEFT)[0];


    const auto foot_down_angles_fr = go2_utils::kinematics::inverse(Eigen::Vector3f(
        -0.1f, go2_utils::robot::L_1, -0.30f
    ), go2_utils::robot::LEFT)[0];

    const auto foot_down_angles_bl = go2_utils::kinematics::inverse(Eigen::Vector3f(
        -0.1f, go2_utils::robot::L_1, -0.30f
    ), go2_utils::robot::LEFT)[0];

    std::cout << "down: " << foot_down_angles_fl << std::endl;

    const auto foot_up_angles = go2_utils::kinematics::inverse(Eigen::Vector3f(
        -0.2f, 0.1f, 0.05f
    ), go2_utils::robot::LEFT)[0];

    std::cout << "up: " << foot_up_angles << std::endl;

    await_data(node, llc);

    for (int i = 0; i < 12; ++i) {
        auto &joint = llc->joint(i);
        joint->cmd().mode = 1;
        joint->cmd().kp = 80;
        joint->cmd().kd = 1;
    }

    llc->frontLeft()->command_joint_angles(foot_down_angles_fl);
    llc->frontRight()->command_joint_angles(foot_down_angles_fr);
    llc->backLeft()->command_joint_angles(foot_down_angles_bl);
    llc->backRight()->command_joint_angles(Eigen::Vector3f(-0.4, 2.8, -1.2));

    const auto timer = node->create_wall_timer(std::chrono::milliseconds(2), [&] {
        llc->publish();
    });

    rclcpp::spin(node);

    rclcpp::shutdown();
}