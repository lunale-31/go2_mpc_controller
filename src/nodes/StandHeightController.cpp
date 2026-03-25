//
// Created by ubuntu on 3/23/26.
//

#include "StandHeightController.h"
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

namespace nodes
{
    StandHeightController::StandHeightController() : Node("stand_height_node")
    {
        low_level_control_ = std::make_shared<interface::LowLevelControl>(this);

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
        timer_ = this->create_wall_timer(1ms, std::bind(&StandHeightController::timer_tick, this));
    }


    void StandHeightController::timer_tick()
    {
        auto &calf = low_level_control_->backRight().calf();

        float q_curr = calf.state().q;

        RCLCPP_INFO(this->get_logger(), "q_curr = %f", q_curr);

        if (target_q_ == INFINITY) {
            move_forwards_ = true;
            target_q_ = max_q;
        }

        if (q_curr > t_max_q) {
            move_forwards_ = false;
            target_q_ = min_q;
        } else if (q_curr < t_min_q) {
            move_forwards_ = true;
            target_q_ = max_q;
        }        

        calf.mode(0x1);
        calf.q(target_q_);
        // calf.q(0.0);
        calf.dq(!move_forwards_ ? dq_pos : dq_neg);
        calf.kp(kp);
        calf.kd(kd);
        calf.tau(tau);
        low_level_control_->publish();
    }
} // nodes