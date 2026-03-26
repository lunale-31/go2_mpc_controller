#include "PidController.h"

namespace common {
    PidController::PidController(const float kp, const float ki, const float kd)
        : kp_(kp), ki_(ki), kd_(kd) {
        reset();
    }

    float PidController::control(const float setpoint, const float current, const float dt) {
        const float error = setpoint - current;
        const float derivative = dt > 0.0f ? (error - prev_error_) / dt : 0.0f;
        acc_error_ += error * dt;
        const float signal = error * kp_ + acc_error_ * ki_ + derivative * kd_; 
        prev_error_ = error;
        return signal * dt;
    }

    void PidController::reset() {
        prev_error_ = 0.0f;
        acc_error_ = 0.0f;
    }
} // namespace common
