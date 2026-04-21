#include <go2_utils/interface/lowlevel/Joint.h>

namespace go2_utils::interface::lowlevel {
    Joint::Joint(unitree_go::msg::MotorCmd &motor_cmd, std::mutex &mtx)
        : motor_cmd_(motor_cmd), mtx_(mtx) {
        // empty
    }

    void Joint::mode(const int val) const {
        std::lock_guard guard(mtx_);
        motor_cmd_.mode = val;
    }

    void Joint::q(const float val) const {
        std::lock_guard guard(mtx_);
        motor_cmd_.q = val;
    }

    void Joint::dq(float val) const {
        std::lock_guard guard(mtx_);
        motor_cmd_.dq = val;
    }

    void Joint::kp(float val) const {
        std::lock_guard guard(mtx_);
        motor_cmd_.kp = val;
    }

    void Joint::kd(float val) const {
        std::lock_guard guard(mtx_);
        motor_cmd_.kd = val;
    }

    void Joint::tau(float val) const {
        std::lock_guard guard(mtx_);
        motor_cmd_.tau = val;
    }

    float Joint::tau() const {
        return motor_cmd_.tau;
    }

    void Joint::state(const unitree_go::msg::MotorState &state) {
        state_ = state;
    }

    unitree_go::msg::MotorState &Joint::state() {
        return state_;
    }
} // namespace go2_utils::interface::lowlevel
