#pragma once

#include "../Controller.h"

namespace controllers::standheight::states {
    /**
     * Motion switcher setter state for the stand height controller state machine
     */
    class MotionSwitcherSet : public Controller::State {
    public:
        MotionSwitcherSet(Controller *controller);
        void timer_tick(Controller *controller) override;

    private:
        std::future<bool> motion_switch_result_;
    };
} // namespace controllers::standheight::states
