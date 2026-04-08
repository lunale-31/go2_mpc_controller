#include "Controller.h"
#include "states/MotionSwitcherSet.h"
#include <chrono>

namespace controllers::standheight {
    Controller::Controller(const rclcpp::Node::SharedPtr &node, std::shared_ptr<Config> &config)
        : node_(node), config_(config) {
        // Initialize interfaces
        low_level_control_ = std::make_shared<interface::LowLevelControl>(node);
        motion_switcher_ = std::make_shared<interface::MotionSwitcher>(node);

        // Initialize timer
        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(config->ms_per_tick), 
            std::bind(&Controller::timer_tick, this)
        );

        /*
        const auto subscription = node->create_subscription<unitree_go::msg::LowCmd>(
            "/lowcmd", 10, [node](const unitree_go::msg::LowCmd &msg) {
            // if (msg.motor_cmd[common::constants::BR_HIP].q == 0.0f) return;

            RCLCPP_INFO(node->get_logger(),
                      "BR Hip: q = %f, dq = %f, tau = %f, kp = %f, kd = %f | "
                      "BR Thigh: q = %f, dq = %f, tau = %f, kp = %f, kd = %f | "
                      "BR Calf: q = %f, dq = %f, tau = %f, kp = %f, kd = %f",
                      msg.motor_cmd[common::constants::BR_HIP].q,
                      msg.motor_cmd[common::constants::BR_HIP].dq,
                      msg.motor_cmd[common::constants::BR_HIP].tau,
                      msg.motor_cmd[common::constants::BR_HIP].kp,
                      msg.motor_cmd[common::constants::BR_HIP].kd,
                      msg.motor_cmd[common::constants::BR_THIGH].q,
                      msg.motor_cmd[common::constants::BR_THIGH].dq,
                      msg.motor_cmd[common::constants::BR_THIGH].tau,
                      msg.motor_cmd[common::constants::BR_THIGH].kp,
                      msg.motor_cmd[common::constants::BR_THIGH].kd,
                      msg.motor_cmd[common::constants::BR_CALF].q,
                      msg.motor_cmd[common::constants::BR_CALF].dq,
                      msg.motor_cmd[common::constants::BR_CALF].tau,
                      msg.motor_cmd[common::constants::BR_CALF].kp,
                      msg.motor_cmd[common::constants::BR_CALF].kd); });

        // */

        // Initialize state machine
        state_ = std::make_shared<states::MotionSwitcherSet>(this);
    }

    void Controller::switch_state(const std::shared_ptr<State> &next) {
        state_ = next;
    }

    rclcpp::Node::SharedPtr &Controller::node() {
        return node_;
    }

    interface::LowLevelControl::SharedPtr &Controller::low_level_control() {
        return low_level_control_;
    }

    interface::MotionSwitcher::SharedPtr &Controller::motion_switcher() {
        return motion_switcher_;
    }

    std::shared_ptr<Config> &Controller::config() {
        return config_;
    }

    void Controller::timer_tick() {
        if (state_) {
            state_->timer_tick(this);
        }
    }

} // namespace controllers::standheight
