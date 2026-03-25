//
// Created by ubuntu on 3/23/26.
//

#ifndef RTSC_UNITREE_ROS2_STANDHEIGHT_H
#define RTSC_UNITREE_ROS2_STANDHEIGHT_H
#include <rclcpp/node.hpp>

#include "../interface/LowLevelControl.h"

namespace controllers
{
    class StandHeightController
    {
    public:
        StandHeightController(const rclcpp::Node::SharedPtr& node);

    private:
        void timer_tick();

        rclcpp::Node::SharedPtr node_;
        rclcpp::TimerBase::SharedPtr timer_;
        interface::LowLevelControl::SharedPtr low_level_control_;

        bool move_forwards_ = false;
        float target_q_ = INFINITY;

        // time
        float t_ = 0;
        const float dt_ = 2 * M_PI / 5 * 0.001;

        // parameters
        float max_q, t_max_q, min_q, t_min_q, dq_pos, dq_neg, kp, kd, tau;
    };
} // nodes

#endif // RTSC_UNITREE_ROS2_STANDHEIGHT_H