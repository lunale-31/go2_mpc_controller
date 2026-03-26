//
// Created by ubuntu on 3/23/26.
//

#include "StandHeightController.h"
#include <chrono>
#include <common/go_constants.h>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

namespace controllers {
    class MotionSwitchRequestState : public StandHeightController::StandHeightState {
    public:
        MotionSwitchRequestState(StandHeightController *controller);
        void timer_tick(StandHeightController *controller) override;

    private:
        std::future<bool> motion_switch_result_;
    };

    class MotionSwitchCheckState : public StandHeightController::StandHeightState {
    public:
        MotionSwitchCheckState(StandHeightController *controller);

        void timer_tick(StandHeightController *controller) override;

    private:
        std::future<bool> motion_switch_check_result_;
    };

    class LegControlState : public StandHeightController::StandHeightState {
    public:
        LegControlState(StandHeightController *controller);

        void timer_tick(StandHeightController *controller) override;

    private:
        bool move_forwards_ = false;
        float target_q_ = INFINITY;

        // time
        float t_ = 0;                          // current time
        const float dt_ = 2 * M_PI / 5 * 0.02; // one revolution every five seconds, at 20ms tick rate

        // parameters
        float max_q, t_max_q, min_q, t_min_q, dq_pos, dq_neg, kp, kd, tau;
    };

    MotionSwitchRequestState::MotionSwitchRequestState(StandHeightController *controller) {
        RCLCPP_INFO(
            controller->node()->get_logger(),
            "Requesting motion to be turned off.");
        motion_switch_result_ = controller->motion_switcher()->set_silent(true);
    }

    void MotionSwitchRequestState::timer_tick(StandHeightController *controller) {
        RCLCPP_INFO(controller->node()->get_logger(), "Waiting...");
        if (motion_switch_result_.wait_for(100ns) == std::future_status::ready) {
            bool success = motion_switch_result_.get();
            RCLCPP_INFO(
                controller->node()->get_logger(),
                "Received response, turning off motion %s.",
                success ? "succeeded" : "failed");
            if (success) {
                controller->switch_state(std::make_shared<MotionSwitchCheckState>(controller));
            } else {
                controller->switch_state(nullptr);
            }
        }
    }

    MotionSwitchCheckState::MotionSwitchCheckState(StandHeightController *controller) {
        RCLCPP_INFO(
            controller->node()->get_logger(),
            "Checking that motion was actually turned off.");
        motion_switch_check_result_ = controller->motion_switcher()->get_silent();
    }

    void MotionSwitchCheckState::timer_tick(StandHeightController *controller)

    {
        RCLCPP_INFO(controller->node()->get_logger(), "Waiting...");
        if (motion_switch_check_result_.wait_for(100ns) == std::future_status::ready) {
            bool silent = motion_switch_check_result_.get();
            RCLCPP_INFO(
                controller->node()->get_logger(),
                "Received response, turning off motion %s.",
                silent ? "succeeded" : "failed");
            if (silent) {
                controller->switch_state(std::make_shared<LegControlState>(controller));
            } else {
                controller->switch_state(nullptr);
            }
        }
    }

    LegControlState::LegControlState(StandHeightController *controller) {
        // Prevent warning
        (void)controller;

        // Load config from params.yaml
        YAML::Node config = YAML::LoadFile("../params.yaml");
        max_q = config["max_q"].as<float>();
        t_max_q = config["t_max_q"].as<float>();
        min_q = config["min_q"].as<float>();
        t_min_q = config["t_min_q"].as<float>();
        dq_pos = config["dq_pos"].as<float>();
        dq_neg = config["dq_neg"].as<float>();
        kp = config["kp"].as<float>();
        kd = config["kd"].as<float>();
        tau = config["tau"].as<float>();
    }

    void LegControlState::timer_tick(StandHeightController *controller) {
        t_ += dt_;
        if (t_ >= 2 * M_PI)
            t_ -= 2 * M_PI;

        float pos = (cos(t_) + 1.0) / 2.0; // yields 0.0 <= pos <= 1.0

        float target_q = (1.0 - pos) * min_q + pos * max_q;

        // old code
        auto &calf = controller->low_level_control()->backRight().calf();
        calf.mode(0x1);
        calf.q(target_q);
        calf.dq(!move_forwards_ ? dq_pos : dq_neg);
        calf.kp(kp);
        calf.kd(kd);
        calf.tau(tau);
        controller->low_level_control()->publish();
    }

    StandHeightController::StandHeightController(const rclcpp::Node::SharedPtr &node) : node_(node) {
        low_level_control_ = std::make_shared<interface::LowLevelControl>(node);
        motion_switcher_ = std::make_shared<interface::MotionSwitcher>(node);

        // Initialize timer
        timer_ = node_->create_wall_timer(20ms, std::bind(&StandHeightController::timer_tick, this));

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
        state_ = std::make_shared<MotionSwitchRequestState>(this);
    }

    void StandHeightController::switch_state(const std::shared_ptr<StandHeightState> &next) {
        state_ = next;
    }

    rclcpp::Node::SharedPtr &StandHeightController::node() {
        return node_;
    }

    interface::LowLevelControl::SharedPtr &StandHeightController::low_level_control() {
        return low_level_control_;
    }

    interface::MotionSwitcher::SharedPtr &StandHeightController::motion_switcher() {
        return motion_switcher_;
    }

    void StandHeightController::timer_tick() {
        if (state_) {
            state_->timer_tick(this);
        }
    }

} // namespace controllers