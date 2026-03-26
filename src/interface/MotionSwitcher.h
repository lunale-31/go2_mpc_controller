#ifndef RTSC_UNITREE_ROS2_MOTIONSWITCHER_H
#define RTSC_UNITREE_ROS2_MOTIONSWITCHER_H

#include "highlevel/UnitreeApi.h"

namespace interface
{
    class MotionSwitcher : highlevel::UnitreeApi
    {
    public:
        explicit MotionSwitcher(const rclcpp::Node::SharedPtr& node);

        std::future<bool> get_silent();
        std::future<bool> set_silent(bool silent);

        using SharedPtr = std::shared_ptr<MotionSwitcher>;
    };
} // interface

#endif //RTSC_UNITREE_ROS2_MOTIONSWITCHER_H
