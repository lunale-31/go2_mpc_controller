#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <go2_utils/interface/LowLevelControl.h>
#include "GaitParams.h"
#include "StandParams.h"

namespace basic_motion::controller {
    class MotionController;
} // namespace basic_motion::controller

namespace basic_motion::controller::states {
    class StateBase {
    protected:
        MotionController *controller_;
        go2_utils::interface::LowLevelControl::SharedPtr llc_;

        rclcpp::Logger get_logger();

    public:
        StateBase(MotionController *controller);
        virtual void enter() = 0;
        virtual void leave() = 0;
        virtual void timer_tick(const float dt) = 0;
        virtual bool transition_damp() = 0;
        virtual bool transition_stand(const StandParams &params) = 0;
        virtual bool transition_gait(const GaitParams &params) = 0;

        using SharedPtr = std::shared_ptr<StateBase>;
    };
} // namespace basic_motion::controller::states
