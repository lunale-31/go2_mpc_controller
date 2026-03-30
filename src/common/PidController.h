#pragma once
#include <memory>

namespace common {
    /**
     * Proportional-integral-derivative controller
     */
    class PidController {
    public:
        /**
         * Constructor.
         * @param kp The proportional gain
         * @param ki The integral gain
         * @param kd The derivative gain
         */
        PidController(const float kp, const float ki, const float kd);

        /**
         * Updates the setpoint.
         * @param setpoint The new setpoint
         */
        void setpoint(const float setpoint);

        /**
         * Gets the current setpoint.
         */
        float setpoint();

        /**
         * Computes the control signal.
         * @param current The current value
         * @param dt The time delta
         */
        float control(const float current, const float dt);

        /**
         * Resets the internal state of the PID controller
         */
        void reset();

        using SharedPtr = std::shared_ptr<PidController>;
        using UniquePtr = std::unique_ptr<PidController>;

    private:
        // PID gains
        const float kp_, ki_, kd_;

        // Previous error
        float prev_error_;

        // Accumulated error
        float acc_error_;

        // Setpoint
        float setpoint_ = 0.0f;
    };
} // namespace common
