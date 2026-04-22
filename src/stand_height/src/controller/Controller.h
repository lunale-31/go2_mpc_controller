#pragma once

#include <rclcpp/rclcpp.hpp>
#include <stand_height/srv/stand_height.hpp>
#include <go2_utils/robot.h>
#include <go2_utils/interface/LowLevelControl.h>

#include "JointPose.h"
#include "JointPoseInterpolation.h"

class Controller {
    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Service<stand_height::srv::StandHeight>::SharedPtr service_;
        rclcpp::TimerBase::SharedPtr timer_;
        go2_utils::interface::LowLevelControl::SharedPtr llc_;

        bool is_initialized_;

        JointPose poses[go2_utils::robot::LEG_COUNT];
        std::unique_ptr<JointPoseInterpolation> pose_interpolators[go2_utils::robot::LEG_COUNT];

    public:
        Controller(const rclcpp::Node::SharedPtr &node);

        // Timer callback
        void timer_tick();

        // Called by the timer during initialization phase
        void initializion_tick();

        // Called by the timer after initialization concluded
        void control_tick();

        void service_request(const stand_height::srv::StandHeight::Request::SharedPtr request,
                          stand_height::srv::StandHeight::Response::SharedPtr response);
};