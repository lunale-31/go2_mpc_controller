#include "Controller.h"
#include "states/MotionSwitcherSet.h"
#include <chrono>

using namespace std::chrono_literals;

namespace controllers::standheight {
    Controller::Controller(const rclcpp::Node::SharedPtr &node) : node_(node) {
        // Initialize interfaces
        low_level_control_ = std::make_shared<interface::LowLevelControl>(node);
        motion_switcher_ = std::make_shared<interface::MotionSwitcher>(node);

        // Initialize timer
        timer_ = node_->create_wall_timer(20ms, std::bind(&Controller::timer_tick, this));

        /*
        auto low_state_subscription_ = node_->create_subscription<unitree_go::msg::LowState>(
            "/lf/lowstate", 10,
            [this](const unitree_go::msg::LowState &msg)
            {

                RCLCPP_INFO(node_->get_logger(),
                            "BL Hip: q = %f, dq = %f, tau = %f    \t"
                            "BL Thigh: q = %f, dq = %f, tau = %f    \t"
                            "BL Calf: q = %f, dq = %f, tau = %f",
                            msg.motor_state[common::constants::BL_HIP].q,
                            msg.motor_state[common::constants::BL_HIP].dq,
                            msg.motor_state[common::constants::BL_HIP].tau_est,
                            msg.motor_state[common::constants::BL_THIGH].q,
                            msg.motor_state[common::constants::BL_THIGH].dq,
                            msg.motor_state[common::constants::BL_THIGH].tau_est,
                            msg.motor_state[common::constants::BL_CALF].q,
                            msg.motor_state[common::constants::BL_CALF].dq,
                            msg.motor_state[common::constants::BL_CALF].tau_est);
            });
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

    void Controller::timer_tick() {
        if (state_) {
            state_->timer_tick(this);
        }
    }

} // namespace controllers::standheight
