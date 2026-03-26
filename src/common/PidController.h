#pragma once

namespace common {
    /**
     * Proportional-integral-derivative controller
     */
    class PidController {
    public:
        /**
         * Constructor
         * @param kp The proportional gain
         * @param ki The integral gain
         * @param kd The derivative gain
         */
        PidController(const float kp, const float ki, const float kd);

        /**
         * Computes the control signal
         * @param setpoint The setpoint (target value)
         * @param current The current value
         * @param dt The time delta
         */
        float control(const float setpoint, const float current, const float dt);

        /**
         * Resets the internal state of the PID controller
         */
        void reset();

    private:
        // PID gains
        const float kp_, ki_, kd_;

        // Previous error
        float prev_error_;

        // Accumulated error
        float acc_error_;
    };
} // namespace common
