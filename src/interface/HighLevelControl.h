#ifndef RTSC_UNITREE_ROS2_HIGHLEVELCONTROL_H
#define RTSC_UNITREE_ROS2_HIGHLEVELCONTROL_H

#include "highlevel/UnitreeApi.h"

namespace interface
{
    class HighLevelControl : highlevel::UnitreeApi
    {
    public:
        explicit HighLevelControl(const rclcpp::Node::SharedPtr& node);

        std::future<bool> damp();
        std::future<bool> stop_move();
        std::future<bool> stand_up();
        std::future<bool> sit_down();

        using SharedPtr = std::shared_ptr<HighLevelControl>;
    };
} // interface

#endif //RTSC_UNITREE_ROS2_HIGHLEVELCONTROL_H
