#include <go2_utils/interact/lowlevel/Joint.h>

namespace go2_utils::interact::lowlevel {
    Joint::Joint(unitree_go::msg::MotorCmd &cmd)
        : cmd_(cmd) {
        // empty
    }

    void Joint::state(const unitree_go::msg::MotorState &state) {
        state_ = state;
    }

    unitree_go::msg::MotorState &Joint::state() {
        return state_;
    }

    unitree_go::msg::MotorCmd & Joint::cmd() {
        return cmd_;
    }
} // namespace interface::lowlevel

