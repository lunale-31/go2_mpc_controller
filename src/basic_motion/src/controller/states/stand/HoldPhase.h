#pragma once

#include "../StandState.h"
#include <eigen3/Eigen/Dense>
#include <go2_utils/robot.h>
#include "../../../util/LinearInterpolator.h"

namespace basic_motion::controller::states::stand {
    class HoldPhase : public StandState::StandStatePhase {
    private:
        StandState *state_;
        MotionController *controller_;
        go2_utils::interface::LowLevelControl::SharedPtr llc_;

        float current_height_;
        std::unique_ptr<util::LinearInterpolator> height_interpolator_;

    public:
        HoldPhase(StandState *state, MotionController *controller,
                  const StandParams &params, const float start_height);
        void set_params(const StandParams &params);
        void timer_tick(const float dt);
        float get_height();
        bool is_transitioning();
    };

} // namespace basic_motion::controller::states::stand
