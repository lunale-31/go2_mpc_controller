#pragma once

#include <rclcpp/node.hpp>

#include "../interface/LowLevelControl.h"
#include "../interface/MotionSwitcher.h"

namespace controllers {
    class StandHeightController {
    public:
        // Base class for state machine states
        class StandHeightState {
        public:
            virtual void timer_tick(StandHeightController *controller) = 0;
        };

        StandHeightController(const rclcpp::Node::SharedPtr &node);
        void switch_state(const std::shared_ptr<StandHeightState> &next);

        rclcpp::Node::SharedPtr &node();
        interface::LowLevelControl::SharedPtr &low_level_control();
        interface::MotionSwitcher::SharedPtr &motion_switcher();

    private:
        void timer_tick();

        // Process state (state machine)
        std::shared_ptr<StandHeightState> state_;

        rclcpp::Node::SharedPtr node_;
        rclcpp::TimerBase::SharedPtr timer_;
        interface::LowLevelControl::SharedPtr low_level_control_;
        interface::MotionSwitcher::SharedPtr motion_switcher_;
    };
} // namespace controllers
