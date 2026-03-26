#include "LowLevelControl.h"

#include <chrono>
#include <go_constants.h>
#include <unitree/go_crc32.h>

static const std::string REQUEST_TOPIC = "/lowcmd";
static const std::string RESPONSE_TOPIC = "/lowstate";

using namespace std::chrono_literals;

namespace interface {
    LowLevelControl::LowLevelControl(const rclcpp::Node::SharedPtr &node) : node_(node) {
        // Initialize low command
        low_command_ = std::make_unique<unitree_go::msg::LowCmd>();
        low_command_->head = {0xFE, 0xEF};
        low_command_->level_flag = 0xFF;
        low_command_->gpio = 0;

        // Initialize publisher and subscriber
        publisher_ = node->create_publisher<unitree_go::msg::LowCmd>(REQUEST_TOPIC, 10);
        subscription_ = node->create_subscription<unitree_go::msg::LowState>(
            RESPONSE_TOPIC, 10,
            std::bind(&LowLevelControl::update_state, this, std::placeholders::_1));

        // Initialize motors and legs
        initialize_motors();
        initialize_legs();
    }

    lowlevel::Leg &LowLevelControl::frontLeft() const {
        return *legs_[common::constants::FRONT_LEFT_INDEX];
    }

    lowlevel::Leg &LowLevelControl::frontRight() const {
        return *legs_[common::constants::FRONT_RIGHT_INDEX];
    }

    lowlevel::Leg &LowLevelControl::backLeft() const {
        return *legs_[common::constants::BACK_LEFT_INDEX];
    }

    lowlevel::Leg &LowLevelControl::backRight() const {
        return *legs_[common::constants::BACK_RIGHT_INDEX];
    }

    void LowLevelControl::initialize_motors() {
        // initialize motors
        for (uint8_t i = 0; i <= common::constants::MOTOR_MAX_INDEX; ++i) {
            motors_[i] = std::make_unique<lowlevel::Motor>(low_command_->motor_cmd[i], command_mtx_);
        }
    }

    void LowLevelControl::initialize_legs() {
        legs_[common::constants::FRONT_LEFT_INDEX] = std::make_unique<lowlevel::Leg>(
            *motors_[common::constants::FL_HIP],
            *motors_[common::constants::FL_THIGH],
            *motors_[common::constants::FL_CALF]);
        legs_[common::constants::FRONT_RIGHT_INDEX] = std::make_unique<lowlevel::Leg>(
            *motors_[common::constants::FR_HIP],
            *motors_[common::constants::FR_THIGH],
            *motors_[common::constants::FR_CALF]);
        legs_[common::constants::BACK_LEFT_INDEX] = std::make_unique<lowlevel::Leg>(
            *motors_[common::constants::BL_HIP],
            *motors_[common::constants::BL_THIGH],
            *motors_[common::constants::BL_CALF]);
        legs_[common::constants::BACK_RIGHT_INDEX] = std::make_unique<lowlevel::Leg>(
            *motors_[common::constants::BR_HIP],
            *motors_[common::constants::BR_THIGH],
            *motors_[common::constants::BR_CALF]);
    }

    void LowLevelControl::update_state(const unitree_go::msg::LowState &state) const {
        // Distribute motor state to motor wrappers
        for (int i = 0; i < common::constants::MOTOR_MAX_INDEX; ++i) {
            this->motors_[i]->state(state.motor_state[i]);
        }
    }

    void LowLevelControl::publish() {
        std::lock_guard guard(command_mtx_);
        // TODO: Prepare and send low-level motor commands here
        RCLCPP_WARN(node_->get_logger(), "Publishing to the robot: tau = %f", low_command_->motor_cmd[common::constants::BR_CALF].tau);
        set_crc(*low_command_);
        publisher_->publish(*low_command_);
    }
} // namespace interface
