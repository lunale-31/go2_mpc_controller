#include "GaitState.h"
#include "../MotionController.h"
#include "DampState.h"
#include "StandState.h"

static constexpr float min_gait_body_height = 0.20f /* meter */;
static constexpr float min_gait_swing_height_distance = 0.08f /* meter */;

namespace basic_motion::controller::states {
    GaitState::GaitState(MotionController *controller, const GaitParams &params)
        : StateBase(controller), params_(params) {
        // Check parameters
        if (params.body_height < min_gait_body_height) {
            throw std::invalid_argument("Body height too low");
        }
        if (params.body_height - params.swing_height < min_gait_swing_height_distance) {
            throw std::invalid_argument("Swing height difference too low");
        }

        // Create leg controllers
        legMotion[go2_utils::robot::LEG_FRONT_LEFT] = std::make_unique<gait::LegMotion>(llc_->frontLeft(), 0);
        legMotion[go2_utils::robot::LEG_BACK_LEFT] = std::make_unique<gait::LegMotion>(llc_->backLeft(), 1);
        legMotion[go2_utils::robot::LEG_BACK_RIGHT] = std::make_unique<gait::LegMotion>(llc_->backRight(), 2);
        legMotion[go2_utils::robot::LEG_FRONT_RIGHT] = std::make_unique<gait::LegMotion>(llc_->frontRight(), 3);
    }

    void GaitState::enter() {
        RCLCPP_INFO(get_logger(), "Entering gaiting state.");
        for (unsigned i = 0; i < go2_utils::robot::LEG_COUNT; ++i) {
            legMotion[i]->start_gaiting(params_);
        }
    }

    void GaitState::leave() {
        RCLCPP_INFO(get_logger(), "Leaving gaiting state.");
    }

    void GaitState::timer_tick(const float dt) {
        for (unsigned i = 0; i < go2_utils::robot::LEG_COUNT; ++i) {
            legMotion[i]->timer_tick(dt);
        }
        llc_->publish();

        // switch to enqueued state if ready
        if (enqueued_state_) {
            bool ready = true;
            for (unsigned i = 0; i < go2_utils::robot::LEG_COUNT; ++i) {
                if (!legMotion[i]->is_standing()) {
                    ready = false;
                    break;
                }
            }
            if (ready) {
                controller_->change_state(enqueued_state_);
                enqueued_state_ = nullptr;
            }
        }
    }

    bool GaitState::transition_damp() {
        auto state = std::make_shared<DampState>(controller_);
        controller_->change_state(state);
        return true;
    }

    bool GaitState::transition_stand(const StandParams &params) {
        RCLCPP_INFO(get_logger(), "Received stop request.");
        try {
            enqueued_state_ = std::make_shared<StandState>(controller_, params);
            for (unsigned i = 0; i < go2_utils::robot::LEG_COUNT; ++i) {
                legMotion[i]->stop_gaiting(params.transition_time);
            }
            return true;
        } catch (std::exception &e) {
            return false;
        }
    }

    bool GaitState::transition_gait(const GaitParams &params) {
        for (unsigned i = 0; i < go2_utils::robot::LEG_COUNT; ++i) {
            legMotion[i]->update_params(params);
        }
        return false;
    }
} // namespace basic_motion::controller::states