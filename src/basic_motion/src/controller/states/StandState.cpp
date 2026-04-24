#include "StandState.h"
#include "../MotionController.h"
#include "DampState.h"
#include "stand/EnterPhase.h"

namespace basic_motion::controller::states {
    
    StandState::StandState(MotionController *controller, const StandParams &params)
    : StateBase(controller) {
        // Initialize enter phase
        phase_ = std::make_shared<stand::EnterPhase>(this, controller, params);
    }
    
    void StandState::enter() {
        RCLCPP_INFO(get_logger(), "Entering standing state.");
        
        // Turn on all motors and use built-in PD controllers
        for (unsigned i = 0; i < go2_utils::robot::JOINT_COUNT; ++i) {
            auto &joint_cmd = llc_->joint(i)->cmd();
            joint_cmd.mode = 1;
            joint_cmd.kp = 40.0f;
            joint_cmd.kd = 5.0f;
            joint_cmd.tau = 0.0f;
        }
    }
    
    void StandState::leave() {
        RCLCPP_INFO(get_logger(), "Leaving standing state.");
    }
    
    void StandState::timer_tick(const float dt) {
        phase_->timer_tick(dt);
        llc_->publish();
    }

    void StandState::change_phase(const std::shared_ptr<StandStatePhase> &next) {
        phase_ = next;
    }

    bool StandState::transition_damp() {
        auto state = std::make_shared<DampState>(controller_);
        controller_->change_state(state);
        return true;
    }

    bool StandState::transition_stand(const StandParams &params) {
        phase_->set_params(params);
        return true;
    }

    bool StandState::transition_gait(const GaitParams &params) {
        (void)params;
        // TODO: Switch to Gait
        return false;
    }
} // namespace basic_motion::controller::states
