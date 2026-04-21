#include "../lib/go2_crc32/go_crc32.h"

#include <chrono>
#include <go2_utils/interface/LowLevelControl.h>
#include <go2_utils/robot.h>

static const std::string REQUEST_TOPIC = "/lowcmd";
static const std::string RESPONSE_TOPIC = "/lowstate";

using namespace std::chrono_literals;

namespace go2_utils::interface {
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

        // Initialize IMU and BMS structs
        imu_state_ = std::make_shared<lowlevel::ImuState>();
        bms_state_ = std::make_shared<lowlevel::BmsState>();
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

    lowlevel::ImuState::SharedPtr &LowLevelControl::imu_state() {
        return imu_state_;
    }

    lowlevel::BmsState::SharedPtr &LowLevelControl::bms_state() {
        return bms_state_;
    }

    void LowLevelControl::initialize_motors() {
        // initialize motors
        for (uint8_t i = 0; i <= robot::JOINT_MAX; ++i) {
            motors_[i] = std::make_shared<lowlevel::Joint>(low_command_->motor_cmd[i], command_mtx_);
        }
    }

    void LowLevelControl::initialize_legs() {
        legs_[robot::LEG_FRONT_LEFT] = std::make_shared<lowlevel::Leg>(
            motors_[robot::JOINT_FL_HIP],
            motors_[robot::JOINT_FL_THIGH],
            motors_[robot::JOINT_FL_CALF]);
        legs_[robot::LEG_FRONT_RIGHT] = std::make_shared<lowlevel::Leg>(
            motors_[robot::JOINT_FR_HIP],
            motors_[robot::JOINT_FR_THIGH],
            motors_[robot::JOINT_FR_CALF]);
        legs_[robot::LEG_BACK_LEFT] = std::make_shared<lowlevel::Leg>(
            motors_[robot::JOINT_BL_HIP],
            motors_[robot::JOINT_BL_THIGH],
            motors_[robot::JOINT_BL_CALF]);
        legs_[robot::LEG_BACK_RIGHT] = std::make_shared<lowlevel::Leg>(
            motors_[robot::JOINT_BR_HIP],
            motors_[robot::JOINT_BR_THIGH],
            motors_[robot::JOINT_BR_CALF]);
    }

    void LowLevelControl::update_state(const unitree_go::msg::LowState &state) const {
        // Distribute motor state to motor wrappers
        for (unsigned i = 0U; i <= robot::JOINT_MAX; ++i) {
            motors_[i]->state(state.motor_state[i]);
        }

        // Update IMU state
        for (int i = 0; i < 4; ++i) {
            imu_state_->quarternion[i] = state.imu_state.quaternion[i];
        }
        imu_state_->roll_velo = state.imu_state.gyroscope[0];
        imu_state_->pitch_velo = state.imu_state.gyroscope[1];
        imu_state_->yaw_velo = state.imu_state.gyroscope[2];
        imu_state_->roll = state.imu_state.rpy[0];
        imu_state_->pitch = state.imu_state.rpy[1];
        imu_state_->yaw = state.imu_state.rpy[2];
        imu_state_->x_acc = state.imu_state.accelerometer[0];
        imu_state_->y_acc = state.imu_state.accelerometer[1];
        imu_state_->z_acc = state.imu_state.accelerometer[2];
        imu_state_->temperature = state.imu_state.temperature;

        // Update BMS state
        bms_state_->version_high = state.bms_state.version_high;
        bms_state_->version_low = state.bms_state.version_low;
        bms_state_->status = static_cast<lowlevel::BmsState::Status>(state.bms_state.status);
        bms_state_->charge_level = state.bms_state.soc;
        bms_state_->charge_cycle = state.bms_state.cycle;
        bms_state_->current = state.bms_state.current;
    }

    void LowLevelControl::publish() {
        std::lock_guard guard(command_mtx_);
        // RCLCPP_WARN(node_->get_logger(), "Publishing to the robot: tau = %f", low_command_->motor_cmd[robot::FR_CALF].tau);
        set_crc(*low_command_);
        publisher_->publish(*low_command_);
    }
} // namespace go2_utils::interface
