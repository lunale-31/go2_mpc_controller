#include "PidController.h"

namespace common {
    PidController::PidController(const float K, const float Ti, const float Td, 
                                 const float N, const float Beta, const float Tr)
        : K_(K), Ti_(Ti), Td_(Td), N_(N), Beta_(Beta), Tr_(Tr) {
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

        // Calculate ad and bd
        const float ad = Td_ / (Td_ + N_ *dt);
        const float bd = K_ * ad * N_;
        
        // Calculate filtered derivative
        D_ = ad * D_ - bd * (current - prev_current_);

        // Calculate proportional term
        const float P = K_ * (Beta_ * setpoint_ - current);

        const float v = P + I_ + D_; 
        
        // Clamp signal to limits to get actual output (u)
        float u = v;

        if (u > signal_max) {
            u = signal_max;
        } else if (u < signal_min) {
            u = signal_min;
        }
        
        // Update integral with tracking anti-windup (back-calculation method)
        if(Ti_ > 0.0f){
            I_ += (K_ * dt / Ti_) * error + (dt / Tr_) * (u-v);
        }

        // Save current value for next iteration
        prev_current_ = current;

        // Return the controller output
        return u;
    }

    void PidController::reset() {
        prev_current_ = 0.0f;
        I_ = 0.0f;
        D_ = 0.0f;
    }
} // namespace common
