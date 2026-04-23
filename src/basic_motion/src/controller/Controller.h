#pragma once

#include <go2_utils/interface/LowLevelControl.h>
#include <go2_utils/robot.h>
#include <rclcpp/rclcpp.hpp>
#include <basic_motion/srv/stand.hpp>

#include "JointPose.h"
#include "JointPoseInterpolation.h"

class Controller {
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Service<basic_motion::srv::Stand>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;
    go2_utils::interface::LowLevelControl::SharedPtr llc_;

    bool is_initialized_;

    JointPose poses[go2_utils::robot::LEG_COUNT];
    std::unique_ptr<JointPoseInterpolation> pose_interpolators[go2_utils::robot::LEG_COUNT];

    void service_request(const basic_motion::srv::Stand::Request::SharedPtr request,
                         basic_motion::srv::Stand::Response::SharedPtr response);

public:
    Controller(const rclcpp::Node::SharedPtr &node);

    // Timer callback
    void timer_tick();

    // Called by the timer during initialization phase
    void initializion_tick();

    // Called by the timer after initialization concluded
    void control_tick();
};