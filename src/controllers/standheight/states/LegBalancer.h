#pragma once

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
        // signal boundaries
        float tau_min_, tau_max_;

        // cascaded PID controllers
        controller::CascadedJointController::UniquePtr hip_controllers_[4];
        controller::CascadedJointController::UniquePtr thigh_controllers_[4];
        controller::CascadedJointController::UniquePtr calf_controllers_[4];

        // controller enabled?
        bool hips_enabled_, thighs_enabled_, calfs_enabled_;

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
