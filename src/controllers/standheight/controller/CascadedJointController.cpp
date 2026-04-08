#include "CascadedJointController.h"
#include <cstdio>

namespace controllers::standheight::controller {

    CascadedJointController::CascadedJointController(interface::lowlevel::Joint::SharedPtr &joint, Config::Joint &config)
        : joint_(joint), config_(config) {
        position_pid_ = std::make_unique<common::PidController>(config.pos_gains.kp, config.pos_gains.ki, config.pos_gains.kd);
        velocity_pid_ = std::make_unique<common::PidController>(config.velo_gains.kp, config.velo_gains.ki, config.velo_gains.kd);
        velocity_pid_->setpoint(0.0);
    }

    void CascadedJointController::setpoint(float pos) {
        position_pid_->setpoint(pos);
    }

    void CascadedJointController::control(float dt) {
        auto state = joint_->state();

        unsigned outer_factor = config_.outer_factor;
        
        if (inner_count_ == 0) {
            const float velo_setpoint = position_pid_->control(state.q, dt * outer_factor);
            velocity_pid_->setpoint(velo_setpoint);
        }

        if (++inner_count_ >= outer_factor) {
            inner_count_ = 0;
        }

        const float torque_signal = velocity_pid_->control(state.dq, dt, config_.tau_min, config_.tau_max);
        /*
        printf(
            "q_curr: %+.4f, q_sp: %+.4f  |  dq_curr: %+.4f, dq_sp: %+.4f  |  tau: %+.4f\n",
            state.q, 
            position_pid_->setpoint(),
            state.dq,
            velocity_pid_->setpoint(),
            torque_signal
        );
        // */
        joint_->mode(1);
        joint_->tau(torque_signal);
    }

} // namespace controllers::standheight::controller
