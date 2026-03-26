//
// Created by ubuntu on 3/23/26.
//

#ifndef RTSC_UNITREE_ROS2_STANDHEIGHT_H
#define RTSC_UNITREE_ROS2_STANDHEIGHT_H
#include <rclcpp/node.hpp>

#include "../interface/LowLevelControl.h"
#include "../interface/MotionSwitcher.h"

namespace controllers
{
    class StandHeightController
    {
    public:
        // Base class for state machine states
        class StandHeightState
        {
        public:
            virtual void timer_tick(StandHeightController *controller) = 0;
        };

        StandHeightController(const rclcpp::Node::SharedPtr &node);
        void switch_state(const std::shared_ptr<StandHeightState> &next);

        rclcpp::Node::SharedPtr& node();
        interface::LowLevelControl::SharedPtr& low_level_control();
        interface::MotionSwitcher::SharedPtr& motion_switcher();

    private:
        void timer_tick();

        // Process state (state machine)
        std::shared_ptr<StandHeightState> state_;

        rclcpp::Node::SharedPtr node_;
        rclcpp::TimerBase::SharedPtr timer_;
        interface::LowLevelControl::SharedPtr low_level_control_;
        interface::MotionSwitcher::SharedPtr motion_switcher_;
    };
} // nodes

#endif // RTSC_UNITREE_ROS2_STANDHEIGHT_H