#pragma once

#include "../Controller.h"

namespace controllers::lowlevel::states {
    /**
     * Motion switcher checker state for the stand height controller state machine
     */
    class MotionSwitcherCheck : public Controller::State {
    public:
        MotionSwitcherCheck(Controller *controller);

        void timer_tick(Controller *controller) override;

    private:
        bool request_sent_ = false;
        std::future<bool> motion_switch_check_result_;
    };
} // namespace controllers::lowlevel::states
