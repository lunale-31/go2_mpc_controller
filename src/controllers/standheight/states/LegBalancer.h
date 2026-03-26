#pragma once

#include "../Controller.h"

namespace controllers::standheight::states {
    /**
     * Leg control state for the stand height controller
     */
    class LegBalancer : public Controller::State {
    public:
        LegBalancer(Controller *controller);

        void timer_tick(Controller *controller) override;

    private:
        bool move_forwards_ = false;
        float target_q_ = INFINITY;

        // time
        float t_ = 0;                          // current time
        const float dt_ = 2 * M_PI / 5 * 0.02; // one revolution every five seconds, at 20ms tick rate

        // parameters
        float max_q, t_max_q, min_q, t_min_q, dq_pos, dq_neg, kp, kd, tau;
    };
} // namespace controllers::standheight::states
