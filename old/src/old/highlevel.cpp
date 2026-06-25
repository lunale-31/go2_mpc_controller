#include <string>
#include <chrono>
#include <nlohmann/json.hpp>
#include <unitree/go_crc32.h>
#include "highlevel.h"
#include "common/go_constants.h"

using namespace std::chrono_literals;

HighLevelNode::HighLevelNode() : Node("high_level_node")
{
    high_command_ = std::make_unique<interface::HighLevelControl>(this->make_shared());

    const bool success = high_command_->sit_down().get();
    RCLCPP_WARN(this->get_logger(), "Sitting down: %s", success ? "success" : "failure");

    /*
    sport_state_subscription_ = this->create_subscription<unitree_go::msg::SportModeState>(
        "/lf/sportmodestate", 10,
        [this](const unitree_go::msg::SportModeState& msg)
        {
            RCLCPP_INFO(this->get_logger(), "Received sport mode state (body height: %f m).",
                        msg.body_height);
        }); // */

    /*
    low_state_subscription_ = this->create_subscription<unitree_go::msg::LowState>(
        "/lf/lowstate", 10,
        [this](const unitree_go::msg::LowState& msg)
        {
            (void)msg;
            (void)this;
            RCLCPP_INFO(this->get_logger(),
                        "BL Hip: q = %f, dq = %f, tau = %f    \t"
                        "BL Thigh: q = %f, dq = %f, tau = %f    \t"
                        "BL Calf: q = %f, dq = %f, tau = %f",
                        msg.motor_state[common::constants::BL_HIP].q,
                        msg.motor_state[common::constants::BL_HIP].dq,
                        msg.motor_state[common::constants::BL_HIP].tau_est,
                        msg.motor_state[common::constants::BL_THIGH].q,
                        msg.motor_state[common::constants::BL_THIGH].dq,
                        msg.motor_state[common::constants::BL_THIGH].tau_est,
                        msg.motor_state[common::constants::BL_CALF].q,
                        msg.motor_state[common::constants::BL_CALF].dq,
                        msg.motor_state[common::constants::BL_CALF].tau_est
            );
        });

    low_cmd_publisher_ = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);
    low_cmd_subscription_ = this->create_subscription<unitree_go::msg::LowCmd>(
        "/lowcmd", 10,
        [this](const unitree_go::msg::LowCmd& msg)
        {
            (void)msg;
            (void)this;
            // std::cout << to_yaml(msg) << std::endl;
            // RCLCPP_INFO(this->get_logger(), "Received low cmd (mode = %d, q_1 = %f, dq_1 = %f, tau_1 = %f).",
            //             msg.motor_cmd[1].mode, msg.motor_cmd[1].q, msg.motor_cmd[1].dq, msg.motor_cmd[1].tau);
        });

    */

    timer_ = this->create_wall_timer(5000ms, [this]
    {
        RCLCPP_INFO(this->get_logger(), "Timer ticked (id = %ld).", id_);

        if (isDown_)
        {
            high_command_->stand_up();
        } else
        {
            high_command_->sit_down();
        }

        /*
        unitree_go::msg::LowCmd cmd;
        cmd.head = {0xFE, 0xEF};
        cmd.level_flag = 0xFF;
        cmd.gpio = 0;
        auto &mcmd = cmd.motor_cmd[common::constants::BL_CALF];
        mcmd.mode = 1;
        mcmd.q = 0;
        mcmd.dq = 0;
        mcmd.kd = 0;
        mcmd.kp = 0;
        mcmd.tau = (isDown_ ? 1 : -1) * 0.1;
        set_crc(cmd);
        low_cmd_publisher_->publish(cmd);
        */

        isDown_ = !isDown_;
        id_++;
    });
}
