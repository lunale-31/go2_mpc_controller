#include "../lib/go2_crc32/go_crc32.h"

#include <chrono>
#include <go2_utils/interact/LowLevelControl.h>
#include <go2_utils/robot.h>

static const std::string REQUEST_TOPIC = "/lowcmd";
static const std::string RESPONSE_TOPIC = "/lowstate";

using namespace std::chrono_literals;

namespace go2_utils::interact {
    LowLevelControl::LowLevelControl(const rclcpp::Node::SharedPtr &node) : node_(node) {
        // Initialize low command
        low_command_ = std::make_unique<unitree_go::msg::LowCmd>();
        low_command_->head = {0xFE, 0xEF};
        low_command_->level_flag = 0xFF;
        low_command_->gpio = 0;

        // Initialize publisher and subscriber
        publisher_ = node->create_publisher<unitree_go::msg::LowCmd>(REQUEST_TOPIC, 10);
        subscription_ = node->create_subscription<unitree_go::msg::LowState>(
            RESPONSE_TOPIC, 10, std::bind(&LowLevelControl::update_state, this, std::placeholders::_1));

        // Initialize motors and legs
        initialize_motors();
        initialize_legs();
    }

    lowlevel::Leg::SharedPtr &LowLevelControl::frontLeft() {
        return legs_[robot::LEG_FRONT_LEFT];
    }

    lowlevel::Leg::SharedPtr &LowLevelControl::frontRight() {
        return legs_[robot::LEG_FRONT_RIGHT];
    }

    lowlevel::Leg::SharedPtr &LowLevelControl::backLeft() {
        return legs_[robot::LEG_BACK_LEFT];
    }

    lowlevel::Leg::SharedPtr &LowLevelControl::backRight() {
        return legs_[robot::LEG_BACK_RIGHT];
    }

    lowlevel::Leg::SharedPtr &LowLevelControl::leg(unsigned index) {
        return legs_[index];
    }

    lowlevel::Joint::SharedPtr &LowLevelControl::joint(unsigned index) {
        return motors_[index];
    }

    unitree_go::msg::IMUState &LowLevelControl::imu_state() {
        return imu_state_;
    }

    unitree_go::msg::BmsState &LowLevelControl::bms_state() {
        return bms_state_;
    }

    void LowLevelControl::initialize_motors() {
        // initialize motors
        for (uint8_t i = 0; i <= robot::JOINT_MAX; ++i) {
            motors_[i] = std::make_shared<lowlevel::Joint>(low_command_->motor_cmd[i]);
        }
    }

    void LowLevelControl::initialize_legs() {
        legs_[robot::LEG_FRONT_LEFT] = std::make_shared<lowlevel::Leg>(
            motors_[robot::JOINT_FL_HIP],
            motors_[robot::JOINT_FL_THIGH],
            motors_[robot::JOINT_FL_CALF],
            robot::LegPair::FRONT, robot::LegSide::LEFT);
        legs_[robot::LEG_FRONT_RIGHT] = std::make_shared<lowlevel::Leg>(
            motors_[robot::JOINT_FR_HIP],
            motors_[robot::JOINT_FR_THIGH],
            motors_[robot::JOINT_FR_CALF],
            robot::LegPair::FRONT, robot::LegSide::RIGHT);
        legs_[robot::LEG_BACK_LEFT] = std::make_shared<lowlevel::Leg>(
            motors_[robot::JOINT_BL_HIP],
            motors_[robot::JOINT_BL_THIGH],
            motors_[robot::JOINT_BL_CALF],
            robot::LegPair::BACK, robot::LegSide::LEFT);
        legs_[robot::LEG_BACK_RIGHT] = std::make_shared<lowlevel::Leg>(
            motors_[robot::JOINT_BR_HIP],
            motors_[robot::JOINT_BR_THIGH],
            motors_[robot::JOINT_BR_CALF],
            robot::LegPair::BACK, robot::LegSide::RIGHT);
    }

    void LowLevelControl::update_state(const unitree_go::msg::LowState &state) {
        // Distribute motor state to motor wrappers
        for (unsigned i = 0U; i <= robot::JOINT_MAX; ++i) {
            motors_[i]->state(state.motor_state[i]);
        }

        // Update IMU state
        imu_state_ = state.imu_state;

        // Update BMS state
        bms_state_ = state.bms_state;

        state_received_ = true;
    }

    void LowLevelControl::publish() {
        set_crc(*low_command_);
        publisher_->publish(*low_command_);
    }

    bool LowLevelControl::was_state_received() {
        return state_received_;
    }
} // namespace go2_utils::interact
