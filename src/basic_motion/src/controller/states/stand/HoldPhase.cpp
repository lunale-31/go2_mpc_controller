#include "HoldPhase.h"
#include "../../MotionController.h"
#include "Pose.h"
#include <algorithm>
#include <go2_utils/robot.h>

namespace basic_motion::controller::states::stand {
    HoldPhase::HoldPhase(StandState *state, MotionController *controller, const StandParams &params, const float start_height)
        : state_(state), controller_(controller), current_height_(start_height) {
        llc_ = controller_->low_level_control();
        set_params(params);
    }

    void HoldPhase::set_params(const StandParams &params) {
        height_interpolator_ = std::make_unique<util::LinearInterpolator>(current_height_, params.body_height, current_height_, params.transition_time);
    }

    void HoldPhase::timer_tick(const float dt) {
        if (height_interpolator_) {
            height_interpolator_->update(dt);
            if (height_interpolator_->finished()) {
                height_interpolator_ = nullptr;
            }
        }

        for (unsigned i = 0; i < go2_utils::robot::LEG_COUNT; ++i) {
            auto &leg = llc_->leg(i);
            auto leg_pose = pose(current_height_, leg->pair(), leg->side());
            leg->command_joint_angles(leg_pose);
        }
    }
    float HoldPhase::get_height() {
        return current_height_;
    }
    bool HoldPhase::is_transitioning() {
        return !!height_interpolator_;
    }
} // namespace basic_motion::controller::states::stand