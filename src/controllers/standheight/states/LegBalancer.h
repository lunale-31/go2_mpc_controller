#pragma once

#include "../Config.h"
#include "../Controller.h"
#include "../controller/CascadedJointController.h"

namespace controllers::standheight::states {
    /**
     * Leg control state for the stand height controller
     */
    class LegBalancer : public Controller::State {
    public:
        LegBalancer(Controller *controller);

        void timer_tick(Controller *controller) override;

    private:
        void update_interpolators(Controller *controller);

        // config
        std::shared_ptr<Config> config_;
        float dt_;
        std::list<Config::MotionStep>::iterator step_iterator_;

        // cascaded PID controllers
        controller::CascadedJointController::UniquePtr hip_controllers_[4];
        controller::CascadedJointController::UniquePtr thigh_controllers_[4];
        controller::CascadedJointController::UniquePtr calf_controllers_[4];

        struct Interpolator {
            float from = NAN, to = NAN;
            float pos_curr = 0.0f, pos_step;
        };
        float time_remaining_;

        Interpolator hip_interpolator[4], thigh_interpolator[4], calf_interpolator[4];
    };
} // namespace controllers::standheight::states
