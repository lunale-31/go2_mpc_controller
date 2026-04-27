#pragma once

#include "../StandState.h"
#include <eigen3/Eigen/Dense>
#include <go2_utils/robot.h>

namespace basic_motion::controller::states::stand {
    class EnterPhase : public StandState::StandStatePhase {
    private:
        StandState *state_;
        MotionController *controller_;
        StandParams params_;
        go2_utils::interface::LowLevelControl::SharedPtr llc_;
        float height_;

        Eigen::Vector3f curr_qs[go2_utils::robot::LEG_COUNT],
            target_qs[go2_utils::robot::LEG_COUNT];

    public:
        EnterPhase(StandState *state, MotionController *controller, const StandParams &params);
        void set_params(const StandParams &params);
        void timer_tick(const float dt);
        float get_height();
        bool is_transitioning();
    };

} // namespace basic_motion::controller::states::stand
