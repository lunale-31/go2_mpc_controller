//
// Created by ubuntu on 3/23/26.
//

#ifndef RTSC_UNITREE_ROS2_MOTOR_H
#define RTSC_UNITREE_ROS2_MOTOR_H

#include <unitree_go/msg/motor_cmd.hpp>
#include <unitree_go/msg/motor_state.hpp>

#include <mutex>

namespace interface::lowlevel {
    class Motor {
    public:
        explicit Motor(unitree_go::msg::MotorCmd &motor_cmd, std::mutex &mtx);

        void state(const unitree_go::msg::MotorState &state);

        unitree_go::msg::MotorState &state();

        void mode(int val) const;

        void q(float val) const;

        void dq(float val) const;

        void kp(float val) const;
        
        void kd(float val) const;

        void tau(float val) const;

    private:
        unitree_go::msg::MotorCmd &motor_cmd_;
        unitree_go::msg::MotorState state_;
        std::mutex &mtx_;
    };
}

#endif //RTSC_UNITREE_ROS2_MOTOR_H
