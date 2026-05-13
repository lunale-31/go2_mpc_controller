#pragma once

#include <rclcpp/node.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>

namespace go2_utils::interact {
    class HighLevelState {
    public:
        /// @brief Constructor
        /// @param node A reference to a ROS2 node used to interface with the robot.
        explicit HighLevelState(const rclcpp::Node::SharedPtr &node);

        /// @brief Returns the robot's lastly-reported trunk position.
        /// @return The trunk position if one has already been received.
        [[nodiscard]] std::array<float, 3> &position();


        /// @brief Returns the robot's lastly-reported trunk velocity.
        /// @return The trunk velocity if one has already been received.
        [[nodiscard]] std::array<float, 3> &velocity();


        /// @brief Returns the robot's lastly-reported foot forces.
        /// @return The foot forces if they have already been received.
        [[nodiscard]] std::array<int16_t, 4> &foot_force();

        /// @brief Returns the robot's lastly-reported IMU state.
        /// @return 
        [[nodiscard]] unitree_go::msg::IMUState &imu_state();

        /// @brief Returns whether the controller has received a low-level state from the robot.
        /// @return True iff a state has been received from the robot.
        [[nodiscard]] bool was_state_received();

    private:
        void update_state(const unitree_go::msg::SportModeState &state);

        // was state received?
        bool state_received_ = false;

        // Motion state
        std::array<float, 3> position_;
        std::array<float, 3> velocity_;

        // Foot force sensors
        std::array<int16_t, 4> foot_force_;

        // IMU state
        unitree_go::msg::IMUState imu_state_;
    };
} // namespace go2_utils::interact
