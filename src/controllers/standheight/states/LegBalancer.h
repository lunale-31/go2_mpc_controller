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
        // signal boundaries
        float tau_min_, tau_max_;

        // joints
        interface::lowlevel::Joint::SharedPtr hip_joints_[4];
        interface::lowlevel::Joint::SharedPtr thigh_joints_[4];
        interface::lowlevel::Joint::SharedPtr calf_joints_[4];

        // pid controllers
        std::unique_ptr<common::PidController> hip_pids_[4];
        std::unique_ptr<common::PidController> thigh_pids_[4];
        std::unique_ptr<common::PidController> calf_pids_[4];
    };
} // namespace controllers::standheight::states
