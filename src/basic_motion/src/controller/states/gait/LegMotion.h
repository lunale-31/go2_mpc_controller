#pragma once

#include "../../../util/LinearInterpolator.h"
#include "../GaitParams.h"
#include <eigen3/Eigen/Dense>
#include <go2_utils/kinematics.h>
#include <go2_utils/robot.h>
#include <go2_utils/interact/lowlevel/Leg.h>

namespace basic_motion::controller::states::gait {
    class LegMotion {
    private:
        go2_utils::interact::lowlevel::Leg::SharedPtr leg_;

        float stand_height_, swing_height_ = 0.0f, swing_forwards_ = 0.0f, swing_backwards_ = 0.0f;
        std::unique_ptr<util::LinearInterpolator>
            stand_height_interpolator_,
            swing_height_interpolator_,
            swing_forwards_interpolator_,
            swing_backwards_interpolator_;

        // The time a full gaiting cycle takes
        float gaiting_period_ = 5.0f, current_time_ = 0.0f;

        // The offset [0, 3] within the cycle
        unsigned offset_;

        // The phase within the gait
        enum GaitPhase {
            STAND_TO_GAIT_TRANSITION,
            GAITING,
            GAIT_TO_STAND_TRANSITION,
            STANDING
        };
        GaitPhase phase_ = GaitPhase::STANDING;

    public:
        LegMotion(const go2_utils::interact::lowlevel::Leg::SharedPtr &leg,
                  unsigned offset);

        void start_gaiting(const GaitParams &params);
        void stop_gaiting(float transition_time);

        void update_params(const GaitParams &params);
        void timer_tick(float dt);

        bool is_standing();
        bool is_gaiting();
    };
} // namespace basic_motion::controller::states::gait