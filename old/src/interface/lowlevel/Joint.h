#pragma once

#include <unitree_go/msg/motor_cmd.hpp>
#include <unitree_go/msg/motor_state.hpp>

#include <mutex>

namespace interface::lowlevel {
    class Joint {
    public:
        /**
         * Constructor.
         * Only to be invoked by the LowLevelControl constructor.
         */
        explicit Joint(unitree_go::msg::MotorCmd &motor_cmd, std::mutex &mtx);

        /**
         * Sets the current state of the joint.
         * Only to be invoked by LowLevelControl.
         */
        void state(const unitree_go::msg::MotorState &state);

        /**
         * Gets the current state of the joint.
         */
        unitree_go::msg::MotorState &state();

        /**
         * Sets the low-level mode of the joint motor.
         * @param val Motor mode: 0x0 to turn it off, 0x1 to turn it on.
         */
        void mode(int val) const;

        /**
         * Sets the desired joint angle, controlled by the internal PD controller.
         * @param val The desired angle, in radians.
         */
        void q(float val) const;

        /**
         * Sets the desired joint angular velocity, controlled by the internal PD controller.
         * @param val The desired angular velocity, in radians per second.
         */
        void dq(float val) const;

        /**
         * Tunes the proportional gain in the internal PD controller.
         * @param val The proportional gain. Set to 0.0f to disable.
         */
        void kp(float val) const;

        /**
         * Tunes the derivative gain in the internal PD controller.
         * @param val The derivative gain. Set to 0.0f to disable.
         */        
        void kd(float val) const;

        /**
         * Sets the torque to apply by the motor.
         * @param val The torque in Newton*meter.
         */
        void tau(float val) const;

        /**
         * Gets the torque set to apply by the motor.
         */
        float tau() const;

        using SharedPtr = std::shared_ptr<Joint>;

    private:
        unitree_go::msg::MotorCmd &motor_cmd_;
        unitree_go::msg::MotorState state_;
        std::mutex &mtx_;
    };
}