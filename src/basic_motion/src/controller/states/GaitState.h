#pragma once

#include "GaitParams.h"
#include "StateBase.h"
#include "gait/LegMotion.h"

namespace basic_motion::controller::states {
    class GaitState : public StateBase {
    private:
        GaitParams params_;
        std::unique_ptr<gait::LegMotion> legMotion[go2_utils::robot::LEG_COUNT];
        
        StateBase::SharedPtr enqueued_state_;
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