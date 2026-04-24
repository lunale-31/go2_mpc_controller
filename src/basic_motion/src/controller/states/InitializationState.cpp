#include "InitializationState.h"
#include "../MotionController.h"
#include "DampState.h"
#include "StandState.h"

namespace basic_motion::controller::states {
    InitializationState::InitializationState(basic_motion::controller::MotionController *controller)
        : StateBase(controller) {
        // empty
    }

    void InitializationState::enter() {
        RCLCPP_INFO(get_logger(), "Entering initialization state.");
    }

    void InitializationState::leave() {
        RCLCPP_INFO(get_logger(), "Leaving initialization state.");
    }

    void InitializationState::timer_tick(const float dt) {
        (void) dt;
        if (!initialization_complete_) {
            if (initialization_complete_ = llc_->was_state_received(), initialization_complete_) {
                controller_->change_state(enqueued_state_);
                enqueued_state_ = nullptr;
            }
        }
    }

    bool InitializationState::transition_damp() {
        auto state = std::make_shared<DampState>(controller_);
        if (initialization_complete_) {
            controller_->change_state(state);
        } else {
            enqueued_state_ = state;
        }
        return true;
    }

    bool InitializationState::transition_stand(const StandParams &params) {
        return true;
    }

    bool InitializationState::transition_gait(const GaitParams &params) {
        (void) params;
        return false;
    }
} // namespace basic_motion::controller::states