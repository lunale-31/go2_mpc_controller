#pragma once

#include <rclcpp/node.hpp>
#include <unitree_go/msg/low_cmd.hpp>
#include <unitree_go/msg/low_state.hpp>

#include "lowlevel/Leg.h"
#include "lowlevel/Joint.h"
#include "lowlevel/BmsState.h"
#include "lowlevel/ImuState.h"

namespace interface {
    class LowLevelControl {
    public:
        /**
         * Constructor.
         * @param node A reference to a ROS2 node used to interface with the robot.
         */
        explicit LowLevelControl(const rclcpp::Node::SharedPtr &node);

        /**
         * Gets the front-left leg.
         */
        [[nodiscard]] lowlevel::Leg::SharedPtr &frontLeft();

        /**
         * Gets the front-right leg.
         */
        [[nodiscard]] lowlevel::Leg::SharedPtr &frontRight();

        /**
         * Gets the back-left leg.
         */
        [[nodiscard]] lowlevel::Leg::SharedPtr &backLeft();

        /**
         * Gets the back-right leg.
         */
        [[nodiscard]] lowlevel::Leg::SharedPtr &backRight();

        /**
         * Gets a leg by index.
         */
        lowlevel::Leg::SharedPtr &leg(unsigned index);

        /**
         * Gets a joint by index.
         */
        lowlevel::Joint::SharedPtr &joint(unsigned index);

        /**
         * Gets the IMU state
         */
        lowlevel::ImuState::SharedPtr &imu_state();

        /**
         * Gets the BMS state
         */
        lowlevel::BmsState::SharedPtr &bms_state();

        /**
         * Sends all current joint commands to the robot. 
         */
        void publish();

        using SharedPtr = std::shared_ptr<LowLevelControl>;

    private:
        void initialize_motors();

        void initialize_legs();

        void update_state(const unitree_go::msg::LowState &state) const;

        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr publisher_;
        rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr subscription_;

        // motors and locks
        lowlevel::Joint::SharedPtr motors_[12];
        lowlevel::Leg::SharedPtr legs_[4];

        // IMU state
        lowlevel::ImuState::SharedPtr imu_state_;
        
        // BMS state
        lowlevel::BmsState::SharedPtr bms_state_;

        // command
        std::mutex command_mtx_;
        std::unique_ptr<unitree_go::msg::LowCmd> low_command_;
    };
} // namespace interface
