#include "PidController.h"

namespace common {
    PidController::PidController(const float kp, const float ki, const float kd)
        : kp_(kp), ki_(ki), kd_(kd) {
        reset();
    }

    void PidController::setpoint(const float setpoint) {
        setpoint_ = setpoint;
    }

    float PidController::setpoint() {
        return setpoint_;
    }

    float PidController::control(const float current, const float dt, const float signal_min, const float signal_max) {
        const float error = setpoint_ - current;
        const float derivative = dt > 0.0f ? (error - prev_error_) / dt : 0.0f;
        float signal = error * kp_ + (acc_error_ + error * dt) * ki_ + derivative * kd_; 
        
        // clamp signal and prevent wind-up
        if (signal > signal_max) {
            signal = signal_max;
        } if (signal < signal_min) {
            signal = signal_min;
        } else {
            acc_error_ += error * dt;
        }
        
        prev_error_ = error;
        return signal;
    }

    void PidController::reset() {
        prev_error_ = 0.0f;
        acc_error_ = 0.0f;
    }
} // namespace common
