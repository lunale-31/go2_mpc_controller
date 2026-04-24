#pragma once

#include "StateBase.h"

namespace basic_motion::controller::states {
    class DampState : public StateBase {
    public:
        DampState(MotionController *controller);
        void enter();
        void leave();
        void timer_tick(const float dt);
        bool transition_damp();
        bool transition_stand(const StandParams &params);
        bool transition_gait(const GaitParams &params);
    };
} // namespace basic_motion::controller::states