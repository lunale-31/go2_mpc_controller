#pragma once

#include "StandParams.h"
#include "StateBase.h"

namespace basic_motion::controller::states {
    class StandState : public StateBase {
    private:
    public:
        class StandStatePhase {
        public:
            virtual void set_params(const StandParams &params) = 0;
            virtual void timer_tick(const float dt) = 0;
        };

        void change_phase(const std::shared_ptr<StandStatePhase> &next);

        StandState(MotionController *controller, const StandParams &params);
        void enter();
        void leave();
        void timer_tick(const float dt);
        bool transition_damp();
        bool transition_stand(const StandParams &params);
        bool transition_gait(const GaitParams &params);

    private:
        std::shared_ptr<StandStatePhase> phase_;
    };
} // namespace basic_motion::controller::states