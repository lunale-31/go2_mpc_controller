#pragma once
#include <memory>
#include <cmath>

namespace common {
    /**
     * Proportional-integral-derivative controller
     */
    class PidController {
    public:
        /**
         * Constructor.
         * @param K The proportional gain
         * @param Ti The integral time constant
         * @param Td The derivative time constant
         * @param N Derivative Filter coefficient 
         * @param Beta Setpoint Weighting
         * @param Tr Tracking anti-windup time constant
         */
        PidController(const float K, const float Ti, const float Td, 
                      const float N, const float Beta, const float Tr);
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
        float control(const float current, const float dt, const float signal_min = -INFINITY, const float signal_max = INFINITY);

        /**
         * Resets the internal state of the PID controller
         */
        void reset();

        using SharedPtr = std::shared_ptr<PidController>;
        using UniquePtr = std::unique_ptr<PidController>;

    private:
        // PID parameters
        const float K_, Ti_, Td_, N_, Beta_, Tr_;

        // Internal state
        float prev_current_; // Previous measurement
        float I_; // Integral term
        float D_; // Derivative term

        // Setpoint
        float setpoint_ = 0.0f;
    };
} // namespace common
