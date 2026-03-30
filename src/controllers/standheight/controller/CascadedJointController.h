#pragma once
#include "../../../common/PidController.h"
#include "../../../interface/lowlevel/Joint.h"

namespace controllers::standheight::controller {

    class CascadedJointController {
    public:
        /**
         * Parameters for the cascaded controller
         */
        struct Config {
            // PID gains for position controller
            float pos_kp, pos_ki, pos_kd;

            // PID gains for velocity controller
            float velo_kp, velo_ki, velo_kd;

            // Bounds for torque
            float tau_min, tau_max;
        };

        CascadedJointController(
            interface::lowlevel::Joint::SharedPtr &joint,
            float pos_setpoint,
            Config &config);

        void control(float dt);

        using SharedPtr = std::shared_ptr<CascadedJointController>;
        using UniquePtr = std::unique_ptr<CascadedJointController>;

    private:
        interface::lowlevel::Joint::SharedPtr joint_;
        common::PidController::UniquePtr position_pid_;
        common::PidController::UniquePtr velocity_pid_;
        float tau_min_, tau_max_; // torque bounds
    };
} // namespace controllers::standheight::controller
