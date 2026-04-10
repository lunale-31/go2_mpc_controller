#pragma once

#include "../Controller.h"
#include "../Config.h"
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
        // config
        std::shared_ptr<Config> config_;

        // cascaded PID controllers
        controller::CascadedJointController::UniquePtr hip_controllers_[4];
        controller::CascadedJointController::UniquePtr thigh_controllers_[4];
        controller::CascadedJointController::UniquePtr calf_controllers_[4];

        struct Interpolator {
            float from, to, pos_curr = 0.0f, pos_step;
        };

        Interpolator hip_interpolator[4], thigh_interpolator[4], calf_interpolator[4];

        /*
        // joints
        interface::lowlevel::Joint::SharedPtr hip_joints_[4];
        interface::lowlevel::Joint::SharedPtr thigh_joints_[4];
        interface::lowlevel::Joint::SharedPtr calf_joints_[4];

        // pid controllers
        common::PidController::UniquePtr hip_pids_[4];
        common::PidController::UniquePtr thigh_pids_[4];
        common::PidController::UniquePtr calf_pids_[4];
        */
    };
} // namespace controllers::standheight::states
