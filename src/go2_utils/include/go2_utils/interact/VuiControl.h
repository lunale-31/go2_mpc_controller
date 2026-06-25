#pragma once

#include "highlevel/UnitreeApi.h"

namespace go2_utils::interact {
    class VuiControl : highlevel::UnitreeApi {
    public:
        /**
         * Constructor.
         * @param node A reference to a ROS2 node used to interface with the robot.
         */
        explicit VuiControl(const rclcpp::Node::SharedPtr &node);


        enum LedColor {
            WHITE,
            RED,
            YELLOW,
            GREEN,
            CYAN,
            BLUE,
            PURPLE,
        };

        /**
         * Requests to change silent mode on or off.
         * @param silent Whether to enable silent mode. If set to true, the robot's internal controller will be turned off upon the next reboot.
         * @returns A future containing whether the command was executed successfully.
         */
        std::future<bool> set_led_color(LedColor color, int time = -1, int flash_cycle = -1);

        using SharedPtr = std::shared_ptr<VuiControl>;
    };
} // namespace interface
