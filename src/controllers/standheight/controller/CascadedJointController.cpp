#include "CascadedJointController.h"

#include <cstdio>

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
        const float velo_setpoint = position_pid_->control(state.q, dt);
        velocity_pid_->setpoint(velo_setpoint);
        const float torque_signal = velocity_pid_->control(state.dq, dt);
        const float bounded_torque = std::clamp(torque_signal, tau_min_, tau_max_);
        printf(
            "q_curr: %+.4f, q_sp: %+.4f, dq_curr: %+.4f, dq_sp: %+.4f, tau_raw: %+.4f, tau_bound %+.4f\n",
            state.q, 
            position_pid_->setpoint(),
            state.dq,
            velo_setpoint,
            torque_signal,
            bounded_torque
        );
        joint_->mode(1);
        joint_->tau(bounded_torque);
    }

} // namespace controllers::standheight::controller
