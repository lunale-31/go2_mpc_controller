#pragma once

#include "../Controller.h"

namespace controllers::lowlevel::states {
    /**
     * Motion switcher setter state for the stand height controller state machine
     */
    class MotionSwitcherSet : public Controller::State {
    public:
        MotionSwitcherSet(Controller *controller);
        void timer_tick(Controller *controller) override;

    private:
        bool request_sent_ = false;
        std::future<bool> motion_switch_result_;
    };
} // namespace controllers::lowlevel::states
