#pragma once

#include "StateBase.h"
#include <go2_utils/interface/LowLevelControl.h>

namespace basic_motion::controller::states {
    class InitializationState : public StateBase {
    private:
        bool initialization_complete_ = false;
        StateBase::SharedPtr enqueued_state_;
    public:
        InitializationState(MotionController *controller);
        void enter();
        void leave();
        void timer_tick(const float dt);
        bool transition_damp();
        bool transition_stand(const StandParams &params);
        bool transition_gait(const GaitParams &params);
    };
} // namespace basic_motion::controller::states