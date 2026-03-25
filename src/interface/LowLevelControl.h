//
// Created by ubuntu on 3/23/26.
//

#ifndef RTSC_UNITREE_ROS2_LOWLEVELCONTROL_H
#define RTSC_UNITREE_ROS2_LOWLEVELCONTROL_H

#include <rclcpp/node.hpp>
#include <unitree_go/msg/low_cmd.hpp>
#include <unitree_go/msg/low_state.hpp>

#include "lowlevel/Motor.h"
#include "lowlevel/Leg.h"

namespace interface {
    class LowLevelControl {
    public:
        explicit LowLevelControl(rclcpp::Node *node);

        [[nodiscard]] lowlevel::Leg &frontLeft() const;

        [[nodiscard]] lowlevel::Leg &frontRight() const;

        [[nodiscard]] lowlevel::Leg &backLeft() const;

        [[nodiscard]] lowlevel::Leg &backRight() const;

        void publish();

        using SharedPtr = std::shared_ptr<LowLevelControl>;

    private:
        void initialize_motors();

        void initialize_legs();

        void update_state(const unitree_go::msg::LowState &state) const;

        rclcpp::Node *node_;
        rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr publisher_;
        rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr subscription_;

        // motors and locks
        std::unique_ptr<lowlevel::Motor> motors_[12];
        std::unique_ptr<lowlevel::Leg> legs_[4];

        // command
        std::mutex command_mtx_;
        std::unique_ptr<unitree_go::msg::LowCmd> low_command_;
    };
} // interface

#endif //RTSC_UNITREE_ROS2_LOWLEVELCONTROL_H
