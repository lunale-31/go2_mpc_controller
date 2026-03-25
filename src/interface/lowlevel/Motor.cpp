//
// Created by ubuntu on 3/23/26.
//

#include "Motor.h"

namespace interface::lowlevel
{
    void Motor::mode(const int val) const
    {
        std::lock_guard guard(mtx_);
        motor_cmd_.mode = val;
    }

    void Motor::q(const float val) const
    {
        std::lock_guard guard(mtx_);
        motor_cmd_.q = val;
    }

    void Motor::dq(float val) const
    {
        std::lock_guard guard(mtx_);
        motor_cmd_.dq = val;
    }

    void Motor::kp(float val) const
    {
        std::lock_guard guard(mtx_);
        motor_cmd_.kp = val;
    }

    void Motor::kd(float val) const
    {
        std::lock_guard guard(mtx_);
        motor_cmd_.kd = val;
    }

    void Motor::tau(float val) const
    {
        std::lock_guard guard(mtx_);
        motor_cmd_.tau = val;
    }

    Motor::Motor(unitree_go::msg::MotorCmd &motor_cmd, std::mutex &mtx) : motor_cmd_(motor_cmd), mtx_(mtx)
    {
        // empty
    }

    void Motor::state(const unitree_go::msg::MotorState &state)
    {
        state_ = state;
    }

    unitree_go::msg::MotorState &Motor::state()
    {
        return state_;
    }
}
