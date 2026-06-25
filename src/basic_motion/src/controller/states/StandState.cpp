#include "StandState.h"
#include "../MotionController.h"
#include "DampState.h"
#include "GaitState.h"
#include "stand/EnterPhase.h"

namespace basic_motion::controller::states {
    StandState::StandState(MotionController *controller, const StandParams &params)
    : StateBase(controller) {
        // Initialize enter phase
        phase_ = std::make_shared<stand::EnterPhase>(this, controller, params);
        enqueued_state_ = nullptr;
    }
    
    void StandState::enter() {
        RCLCPP_INFO(get_logger(), "Entering standing state.");
        
        // Turn on all motors and use built-in PD controllers
        for (unsigned i = 0; i < go2_utils::robot::JOINT_COUNT; ++i) {
            auto &joint_cmd = llc_->joint(i)->cmd();
            joint_cmd.mode = 1;
            joint_cmd.kp = 80.0f;
            joint_cmd.kd = 6.0f;
            joint_cmd.tau = 0.0f;
        }
    }
    
    void StandState::leave() {
        RCLCPP_INFO(get_logger(), "Leaving standing state.");
    }
    
    void StandState::timer_tick(const float dt) {
        phase_->timer_tick(dt);
        llc_->publish();

        if (next_phase_) {
            phase_ = next_phase_;
            next_phase_ = nullptr;
        }

        if (enqueued_state_ && !phase_->is_transitioning()) {
            controller_->change_state(enqueued_state_);
            enqueued_state_ = nullptr;   
        }
    }

    void StandState::change_phase(const std::shared_ptr<StandStatePhase> &next) {
        next_phase_ = next;
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
        try {
            // Constructor will raise an exception if params are invalid
            enqueued_state_ = std::make_shared<GaitState>(controller_, params);
            // Before switching to gait state, reach desired height
            StandParams stand_params {
                .body_height = params.body_height,
                .transition_time = params.transition_time / 3.0f
            };
            transition_stand(stand_params);
            return true;
        } catch (std::exception &e) {
            return false;
        }
    }
} // namespace basic_motion::controller::states
