#include "CascadedJointController.h"
#include <cstdio>

#define INNER_LOOP_FACTOR 10

namespace controllers::standheight::controller {

    CascadedJointController::CascadedJointController(interface::lowlevel::Joint::SharedPtr &joint, float pos_setpoint, Config &config)
        : joint_(joint), tau_min_(config.tau_min), tau_max_(config.tau_max) {
        position_pid_ = std::make_unique<common::PidController>(config.pos_kp, config.pos_ki, config.pos_kd);
        position_pid_->setpoint(pos_setpoint);
        velocity_pid_ = std::make_unique<common::PidController>(config.velo_kp, config.velo_ki, config.velo_kd);
        velocity_pid_->setpoint(0.0);
    }

    void CascadedJointController::control(float dt) {
        auto state = joint_->state();
        
        if (inner_count_ == 0) {
            const float velo_setpoint = position_pid_->control(state.q, dt * INNER_LOOP_FACTOR);
            velocity_pid_->setpoint(velo_setpoint);
            // velocity_pid_->setpoint(0.3);
        }

        if (++inner_count_ == INNER_LOOP_FACTOR) {
            inner_count_ = 0;
        }

        const float torque_signal = velocity_pid_->control(state.dq, dt, tau_min_, tau_max_);
        printf(
            "q_curr: %+.4f, q_sp: %+.4f  |  dq_curr: %+.4f, dq_sp: %+.4f  |  tau: %+.4f\n",
            state.q, 
            position_pid_->setpoint(),
            state.dq,
            velocity_pid_->setpoint(),
            torque_signal
        );
        joint_->mode(1);
        joint_->tau(torque_signal);
    }

} // namespace controllers::standheight::controller
