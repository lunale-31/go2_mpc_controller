#pragma once

#include "highlevel/UnitreeApi.h"

namespace go2_utils::interact {
    class MotionSwitcher : highlevel::UnitreeApi {
    public:
        /**
         * Constructor.
         * @param node A reference to a ROS2 node used to interface with the robot.
         */
        explicit MotionSwitcher(const rclcpp::Node::SharedPtr &node);

        /**
         * Queries whether the robot is set to silent mode.
         * @returns A future containing whether the robot is set to silent mode.
         */
        std::future<bool> get_silent();

        /**
         * Requests to change silent mode on or off.
         * @param silent Whether to enable silent mode. If set to true, the robot's internal controller will be turned off upon the next reboot.
         * @returns A future containing whether the command was executed successfully.
         */
        std::future<bool> set_silent(bool silent);

        using SharedPtr = std::shared_ptr<MotionSwitcher>;
    };
} // namespace interface
