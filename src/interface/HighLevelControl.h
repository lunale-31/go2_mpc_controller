#ifndef RTSC_UNITREE_ROS2_HIGHLEVELCONTROL_H
#define RTSC_UNITREE_ROS2_HIGHLEVELCONTROL_H

#include "highlevel/UnitreeApi.h"

namespace interface
{
    class HighLevelControl : highlevel::UnitreeApi
    {
    public:
        explicit HighLevelControl(const rclcpp::Node::SharedPtr& node);

        std::future<unitree_api::msg::Response::SharedPtr> damp();
        std::future<unitree_api::msg::Response::SharedPtr> stop_move();
        std::future<unitree_api::msg::Response::SharedPtr> stand_up();
        std::future<unitree_api::msg::Response::SharedPtr> sit_down();
    };
} // interface

#endif //RTSC_UNITREE_ROS2_HIGHLEVELCONTROL_H
