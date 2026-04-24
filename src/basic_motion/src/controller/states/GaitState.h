#pragma once

#include "GaitParams.h"
#include "StateBase.h"

namespace basic_motion::controller::states {
    class GaitState : public StateBase {
    public:
        GaitState(MotionController *controller, const GaitParams &params);
        void enter();
        void leave();
        void timer_tick(const float dt);
        bool transition_damp();
        bool transition_stand(const StandParams &params);
        bool transition_gait(const GaitParams &params);
    };
} // namespace basic_motion::controller::states