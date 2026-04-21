#pragma once
#include "../../../common/PidController.h"
#include "../../../interface/lowlevel/Joint.h"
#include "../Config.h"

namespace controllers::lowlevel::controller {

    class CascadedJointController {
    public:

        CascadedJointController(
            interface::lowlevel::Joint::SharedPtr &joint,
            Config::Joint &config);

        void setpoint(float pos);
        float setpoint();

        float current();

        float signal();
        
        void control(float dt);

        using SharedPtr = std::shared_ptr<CascadedJointController>;
        using UniquePtr = std::unique_ptr<CascadedJointController>;

    private:
        interface::lowlevel::Joint::SharedPtr joint_;
        Config::Joint &config_;

        common::PidController::UniquePtr position_pid_;
        common::PidController::UniquePtr velocity_pid_;

        float q_ = NAN, dq_ = NAN;

        unsigned inner_count_ = 0;
    };
} // namespace controllers::lowlevel::controller
