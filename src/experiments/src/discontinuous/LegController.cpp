#include "LegController.h"
#include <cassert>
#include <go2_utils/kinematics.h>
#include <go2_utils/robot.h>

// Acceptable range of thigh angles for gaiting
static constexpr float thigh_min = 0.0f, thigh_max = 2.5f;

void LegController::advance_step_boundaries() {
    step_start_ = step_end_;
    if (current_step_is_push()) {
        step_end_ = static_cast<FootPosition>(step_start_ + 1);
    } else if (current_step_ == swing_step_) {
        step_end_ = FORWARDS;
    }
}

Eigen::Vector3f LegController::joint_angles_for_foot_position(const float x, const float z) {
    for (const auto &angles : go2_utils::kinematics::inverse(
             Eigen::Vector3f(
                 x,
                 leg_->side() == go2_utils::robot::LEFT ? go2_utils::robot::L_1 : -go2_utils::robot::L_1,
                 z),
             leg_->side())) {
        if (angles.y() >= thigh_min && angles.y() <= thigh_max) {
            return angles;
        }
    }
    throw std::runtime_error("Could not find valid joint angles.");
}

inline bool LegController::current_step_is_push() {
    for (size_t i = 0; i < (sizeof(push_steps) / sizeof(*push_steps)); ++i) {
        if (current_step_ == push_steps[i]) {
            return true;
        }
    }
    return false;
}

void LegController::perform_swing() {
    assert(step_start_ == BACKWARDS && step_end_ == FORWARDS);
    const auto x_pos = (1.0f - step_progress_) * x_positions(step_start_) + step_progress_ * x_positions(step_end_);
    const auto z_progress = -4.0f * std::pow(step_progress_, 2.0f) + 4.0f * step_progress_;
    const auto z_pos = (1.0f - z_progress) * z_stand + z_progress * z_swing;
    const auto angles = joint_angles_for_foot_position(x_pos, z_pos);
    leg_->command_joint_angles(angles);
}

void LegController::perform_push() {
    assert(step_start_ + 1 == step_end_);
    const auto step_progress_smoothened = 3.0f * std::pow(step_progress_, 2.0f) - 2.0f * std::pow(step_progress_, 3.0f);
    const auto x_pos = (1.0f - step_progress_smoothened) * x_positions(step_start_) + step_progress_smoothened * x_positions(step_end_);
    const auto angles = joint_angles_for_foot_position(x_pos, z_stand);
    leg_->command_joint_angles(angles);
}

void LegController::perform_stand() {
    assert(step_start_ == step_end_);
    const auto angles = joint_angles_for_foot_position(x_positions(step_start_), z_stand);
    leg_->command_joint_angles(angles);
    const auto kp_progress = -4.0f * std::pow(step_progress_, 2.0f) + 4.0f * step_progress_;
    leg_set_kps((1.0f - kp_progress) * 80 + kp_progress * 140);
}

void LegController::leg_set_kps(float kp) {
    leg_->hip()->cmd().kp = kp;
    leg_->thigh()->cmd().kp = kp;
    leg_->calf()->cmd().kp = kp;
}

LegController::LegController(go2_utils::interact::lowlevel::Leg::SharedPtr &leg, int swing_step, FootPosition first_step_start, FootPosition first_step_end)
    : leg_(leg), swing_step_(swing_step), step_start_(first_step_start), step_end_(first_step_end) {
    // nothing to do here
}

void LegController::tick() {
    // perform action for current step
    if (current_step_ == swing_step_) {
        perform_swing();
    } else if (current_step_is_push()) {
        perform_push();
    } else {
        perform_stand();
    }

    // advance steps
    step_progress_ += (1 / step_length_);
    if (step_progress_ >= 1.0f) {
        step_progress_ = 0.0f;
        if (++current_step_ >= count_step_) {
            current_step_ = 0;
        }
        advance_step_boundaries();
    }
}