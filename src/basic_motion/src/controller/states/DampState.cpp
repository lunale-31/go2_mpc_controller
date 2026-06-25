#include "DampState.h"
#include "../MotionController.h"
#include "StandState.h"
#include <go2_utils/robot.h>

namespace basic_motion::controller::states {
    DampState::DampState(MotionController *controller)
        : StateBase(controller) {
    }

    void DampState::enter() {
        RCLCPP_INFO(get_logger(), "Entering damping state.");

        // Turn off all motors
        for (unsigned i = 0; i < go2_utils::robot::JOINT_COUNT; ++i) {
            auto &joint_cmd = llc_->joint(i)->cmd();
            joint_cmd.mode = 0;
            joint_cmd.kp = 0.0f;
            joint_cmd.kd = 0.0f;
            joint_cmd.tau = 0.0f;
        }
    }

    void DampState::leave() {
        RCLCPP_INFO(get_logger(), "Leaving damping state.");
    }

    void DampState::timer_tick(const float dt) {
        (void) dt;
        llc_->publish();
    }

    bool DampState::transition_damp() {
        // already in damping state
        return true;
    }

    bool DampState::transition_stand(const StandParams &params) {
        auto state = std::make_shared<StandState>(controller_, params);
        controller_->change_state(state);
        return true;
    }

    bool DampState::transition_gait(const GaitParams &params) {
        (void)params;
        return false;
    }
} // namespace basic_motion::controller::states
