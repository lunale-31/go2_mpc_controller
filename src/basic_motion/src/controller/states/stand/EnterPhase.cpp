#include "EnterPhase.h"
#include "../../MotionController.h"
#include "HoldPhase.h"
#include "Pose.h"
#include <algorithm>
#include <go2_utils/robot.h>

static constexpr float dq_max[]{0.3f, 0.3f, 0.3f} /* rad/s */;

static constexpr float stand_height_min = 0.0f /* meter */;
static constexpr float stand_height_max = 0.35f /* meter */;

static constexpr float q_epsilon = 0.01 /* rad */;

namespace basic_motion::controller::states::stand {
    EnterPhase::EnterPhase(StandState *state, MotionController *controller, const StandParams &params)
        : state_(state), controller_(controller), params_(params) {
        llc_ = controller_->low_level_control();

        // Estimate height
        height_ = 0.0;
        for (unsigned i = 0; i < go2_utils::robot::LEG_COUNT; ++i) {
            height_ += std::clamp(-llc_->leg(i)->foot_position().z(),
                                 stand_height_min, stand_height_max);
        }
        height_ /= 4.0f;

        // Prepare joint transition
        for (unsigned i = 0; i < go2_utils::robot::LEG_COUNT; ++i) {
            auto &leg = llc_->leg(i);
            curr_qs[i] = leg->joint_angles();
            target_qs[i] = pose(height_, leg->pair(), leg->side());
        }
    }

    void EnterPhase::set_params(const StandParams &params) {
        params_ = params;
    }

    void EnterPhase::timer_tick(const float dt) {
        bool target_reached = true;
        for (unsigned j = 0; j < 3; ++j) {
            const float step = dt * dq_max[j];
            for (unsigned i = 0; i < go2_utils::robot::LEG_COUNT; ++i) {
                curr_qs[i][j] = std::clamp(target_qs[i][j], curr_qs[i][j] - step, curr_qs[i][j] + step);
                if (abs(curr_qs[i][j] - target_qs[i][j]) > q_epsilon) {
                    target_reached = true;
                }
            }
        }
        for (unsigned i = 0; i < go2_utils::robot::LEG_COUNT; ++i) {
            llc_->leg(i)->command_joint_angles(curr_qs[i]);
        }
        llc_->publish();
        if (target_reached) {
            RCLCPP_INFO(controller_->get_logger(), "Reached initial height of %.4f.", height_);
            auto next_state = std::make_shared<HoldPhase>(state_, controller_, params_, height_);
            state_->change_phase(next_state);
        }
    }
} // namespace basic_motion::controller::states::stand
