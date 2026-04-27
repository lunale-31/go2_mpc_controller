#include "InitializationState.h"
#include "../MotionController.h"
#include "DampState.h"
#include "StandState.h"

namespace basic_motion::controller::states {
    void InitializationState::change_state_if_ready() {
        if (initialization_complete_ && enqueued_state_) {
            controller_->change_state(enqueued_state_);
            enqueued_state_ = nullptr;
        }
    }

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
        (void)dt;
        if (llc_->was_state_received()) {
            initialization_complete_ = true;
            change_state_if_ready();
        } else {
            RCLCPP_INFO(get_logger(), "Waiting for robot.");
        }
    }

    bool InitializationState::transition_damp() {
        enqueued_state_ = std::make_shared<DampState>(controller_);
        change_state_if_ready();
        return true;
    }

    bool InitializationState::transition_stand(const StandParams &params) {
        enqueued_state_ = std::make_shared<StandState>(controller_, params);
        change_state_if_ready();
        return true;
    }

    bool InitializationState::transition_gait(const GaitParams &params) {
        (void)params;
        // we do not allow this transition
        return false;
    }
} // namespace basic_motion::controller::states