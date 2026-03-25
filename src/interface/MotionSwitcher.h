#ifndef RTSC_UNITREE_ROS2_HIGHLEVELCONTROL_H
#define RTSC_UNITREE_ROS2_HIGHLEVELCONTROL_H

#include "highlevel/UnitreeApi.h"

namespace interface
{
    class MotionSwitcher : highlevel::UnitreeApi
    {
    public:
        explicit MotionSwitcher(const rclcpp::Node::SharedPtr& node);

        std::future<const unitree_api::msg::Response::SharedPtr> get_silent();
        std::future<const unitree_api::msg::Response::SharedPtr> set_silent(bool silent);
    };
} // interface

#endif //RTSC_UNITREE_ROS2_HIGHLEVELCONTROL_H
