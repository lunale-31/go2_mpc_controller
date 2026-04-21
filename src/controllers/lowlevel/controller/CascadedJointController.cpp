#include "CascadedJointController.h"
#include <cstdio>

namespace controllers::lowlevel::controller {

    CascadedJointController::CascadedJointController(interface::lowlevel::Joint::SharedPtr &joint, Config::Joint &config)
        : joint_(joint), config_(config) {
        position_pid_ = std::make_unique<common::PidController>(
            config.pos_gains.K, config.pos_gains.Ti, config.pos_gains.Td,
            config.pos_gains.N, config.pos_gains.Beta, config.pos_gains.Tr);
        velocity_pid_ = std::make_unique<common::PidController>(
            config.velo_gains.K, config.velo_gains.Ti, config.velo_gains.Td,
            config.velo_gains.N, config.velo_gains.Beta, config.velo_gains.Tr);
        velocity_pid_->setpoint(0.0);
    }

    void CascadedJointController::setpoint(float pos) {
        position_pid_->setpoint(pos);
        joint_->q(pos);
    }

    float CascadedJointController::setpoint() {
        return position_pid_->setpoint();
    }

    float CascadedJointController::current() {
        return q_;
    }

    float CascadedJointController::signal() {
        return joint_->tau();
    }

    void CascadedJointController::control(float dt) {
        auto state = joint_->state();

        unsigned outer_factor = config_.outer_factor;

        const float b_q = dt / (config_.pos_gains.Tf * 0.001f);
        const float b_dq = dt / (config_.velo_gains.Tf * 0.001f);

        dq_ = std::isnan(dq_) ? state.dq : (1 - b_dq) * dq_ + b_dq * state.dq;
        q_ = std::isnan(q_) ? state.q : (1 - b_q) * q_ + b_q * state.q;

        if (inner_count_ == 0) {
            const float velo_setpoint = position_pid_->control(q_, dt * outer_factor);
            velocity_pid_->setpoint(velo_setpoint);
            joint_->dq(velo_setpoint);
        }

        if (++inner_count_ >= outer_factor) {
            inner_count_ = 0;
        }

        const float torque_signal = velocity_pid_->control(dq_, dt, config_.tau_min, config_.tau_max);
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
        joint_->kp(0.0f);
        joint_->kd(0.0f);
        joint_->tau(torque_signal);
    }

} // namespace controllers::lowlevel::controller
