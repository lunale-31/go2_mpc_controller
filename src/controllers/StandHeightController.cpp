//
// Created by ubuntu on 3/23/26.
//

#include "StandHeightController.h"
#include <yaml-cpp/yaml.h>
#include <common/go_constants.h>

using namespace std::chrono_literals;

namespace controllers
{
    StandHeightController::StandHeightController(const rclcpp::Node::SharedPtr& node) : node_(node)
    {
        low_level_control_ = std::make_shared<interface::LowLevelControl>(node);

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

        // Initialize timer
        timer_ = node_->create_wall_timer(20ms, std::bind(&StandHeightController::timer_tick, this));

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
    }

    void StandHeightController::timer_tick()
    {
        t_ += dt_;
        if (t_ >= 2 * M_PI) t_ -= 2 * M_PI;

        float pos = (cos(t_) + 1.0) / 2.0; // yields 0.0 <= pos <= 1.0

        float target_q = (1.0 - pos) * min_q + pos * max_q;



        // old code
        auto &calf = low_level_control_->backRight().calf();

        /*
        float q_curr = calf.state().q;

        RCLCPP_INFO(this->get_logger(), "q_curr = %f", q_curr);

        if (target_q_ == INFINITY)
        {
            move_forwards_ = true;
            target_q_ = max_q;
        }

        if (q_curr > t_max_q)
        {
            move_forwards_ = false;
            target_q_ = min_q;
        }
        else if (q_curr < t_min_q)
        {
            move_forwards_ = true;
            target_q_ = max_q;
        }

        */

        calf.mode(0x1);
        calf.q(target_q);
        // calf.q(0.0);
        calf.dq(!move_forwards_ ? dq_pos : dq_neg);
        calf.kp(kp);
        calf.kd(kd);
        calf.tau(tau);
        low_level_control_->publish();
    }
} // nodes