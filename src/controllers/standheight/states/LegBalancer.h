#pragma once

#include "../Controller.h"
#include "../../../common/PidController.h"

namespace controllers::standheight::states {
    /**
     * Leg control state for the stand height controller
     */
    class LegBalancer : public Controller::State {
    public:
        LegBalancer(Controller *controller);

        void timer_tick(Controller *controller) override;

    private:
        // pid controller
        float setpoint_;
        std::unique_ptr<common::PidController> pid_;

        // signal boundaries
        float tau_min_, tau_max_;
    };
} // namespace controllers::standheight::states
