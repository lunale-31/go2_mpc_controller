//
// Created by ubuntu on 3/23/26.
//

#include "StandHeightController.h"

using namespace std::chrono_literals;

namespace nodes
{
    StandHeightController::StandHeightController() : Node("stand_height_node")
    {
        low_level_control_ = std::make_shared<interface::LowLevelControl>(this);
        timer_ = this->create_wall_timer(1ms, [this] {
            if (target_q_ == INFINITY) {
                target_q_ = low_level_control_->backRight().calf().state().q;
            }

            target_q_ += (move_forwards_ ? 0.0003 : -0.0003);
            if (constexpr float max_q = -0.8; target_q_ > max_q) {
                target_q_ = max_q;
                move_forwards_ = false;
            } else if (constexpr float min_q = -2.7; target_q_ < min_q) {
                target_q_ = min_q;
                move_forwards_ = true;
            }

            auto &calf = low_level_control_->backRight().calf();
            calf.mode(0x1);
            calf.q(target_q_);
            // calf.q(0.0);
            calf.dq(move_forwards_ ? 0.2 : -0.2);
            calf.kp(60.0);
            calf.kd(5.0);
            calf.tau(0.0);
            low_level_control_->publish();
        });
    }
} // nodes