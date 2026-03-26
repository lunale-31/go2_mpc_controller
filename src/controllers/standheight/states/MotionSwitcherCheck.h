#pragma once

#include "../Controller.h"

namespace controllers::standheight::states {
    /**
     * Motion switcher checker state for the stand height controller state machine
     */
    class MotionSwitcherCheck : public Controller::State {
    public:
        MotionSwitcherCheck(Controller *controller);

        void timer_tick(Controller *controller) override;

    private:
        std::future<bool> motion_switch_check_result_;
    };
} // namespace controllers::standheight::states
