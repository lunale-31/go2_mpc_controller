#include "go2_utils/interact/HighLevelState.h"

static const std::string DDS_TOPIC = "/sportmodestate";

namespace go2_utils::interact {
    HighLevelState::HighLevelState(const rclcpp::Node::SharedPtr &node) {
        node->create_subscription(DDS_TOPIC, 10, std::bind(&HighLevelState::update_state, this, std::placeholders::_1));
    }

    std::array<float, 3> &HighLevelState::position() {
        return position_;
    }

    std::array<float, 3> &HighLevelState::velocity() {
        return velocity_;
    }

    std::array<int16_t, 4> &HighLevelState::foot_force() {
        return foot_force_;
    }

    unitree_go::msg::IMUState &HighLevelState::imu_state() {
        return imu_state_;
    }

    bool HighLevelState::was_state_received() {
        return state_received_;
    }

    void HighLevelState::update_state(const unitree_go::msg::SportModeState &state) {
        position_ = state.position;
        velocity_ = state.velocity;
        foot_force_ = state.foot_force;
        imu_state_ = state.imu_state;
        state_received_ = true;
    }

} // namespace go2_utils::interact
