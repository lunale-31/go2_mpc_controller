#pragma once
#include "../../../common/PidController.h"
#include "../../../interface/lowlevel/Joint.h"
#include "../Config.h"

namespace controllers::standheight::controller {

    class CascadedJointController {
    public:

        CascadedJointController(
            interface::lowlevel::Joint::SharedPtr &joint,
            Config::Joint &config);

        void setpoint(float pos);
        
        void control(float dt);

        using SharedPtr = std::shared_ptr<CascadedJointController>;
        using UniquePtr = std::unique_ptr<CascadedJointController>;

    private:
        interface::lowlevel::Joint::SharedPtr joint_;
        Config::Joint &config_;

        common::PidController::UniquePtr position_pid_;
        common::PidController::UniquePtr velocity_pid_;

        unsigned inner_count_ = 0;
    };
} // namespace controllers::standheight::controller
