#ifndef HIGHLEVEL_H
#define HIGHLEVEL_H
#include <rclcpp/rclcpp.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>
#include <unitree_go/msg/low_cmd.hpp>
#include <unitree_go/msg/low_state.hpp>

#include "interface/HighLevelControl.h"

class HighLevelNode : public rclcpp::Node {
public:
    HighLevelNode();

private:
    std::unique_ptr<interface::HighLevelControl> high_command_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_state_subscription_;

    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr low_cmd_publisher_;
    rclcpp::Subscription<unitree_go::msg::LowCmd>::SharedPtr low_cmd_subscription_;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_subscription_;


    bool isDown_ = false;
    long int id_ = 9000;
};
#endif // HIGHLEVEL_H
