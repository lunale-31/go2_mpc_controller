#include "LegMotion.h"
#include <rclcpp/rclcpp.hpp>

static constexpr float thigh_min = 0.0f, thigh_max = 2.5f;

namespace basic_motion::controller::states::gait {
    LegMotion::LegMotion(const go2_utils::interface::lowlevel::Leg::SharedPtr &leg,
                         unsigned offset)
        : leg_(leg), offset_(offset) {
    }
    void LegMotion::start_gaiting(const GaitParams &params) {
        // Estimate current stand height from leg
        stand_height_ = -leg_->foot_position().z();

        switch (phase_) {
            case GaitPhase::STAND_TO_GAIT_TRANSITION:
            case GaitPhase::GAITING:
                return;
            case GaitPhase::GAIT_TO_STAND_TRANSITION:
            case GaitPhase::STANDING:
                phase_ = STAND_TO_GAIT_TRANSITION;
                break;
        }

        update_params(params);
    }

    void LegMotion::stop_gaiting(float transition_time) {
        switch (phase_) {
            case GaitPhase::GAIT_TO_STAND_TRANSITION:
            case GaitPhase::STANDING:
                return;
            case GaitPhase::STAND_TO_GAIT_TRANSITION:
            case GaitPhase::GAITING:
                phase_ = GAIT_TO_STAND_TRANSITION;
                break;
        }

        stand_height_interpolator_ = std::make_unique<util::LinearInterpolator>(stand_height_,
                                                                                stand_height_,
                                                                                stand_height_,
                                                                                transition_time);
        swing_height_interpolator_ = std::make_unique<util::LinearInterpolator>(swing_height_,
                                                                                0.0f,
                                                                                swing_height_,
                                                                                transition_time);
        swing_forwards_interpolator_ = std::make_unique<util::LinearInterpolator>(swing_forwards_,
                                                                                  0.0f,
                                                                                  swing_forwards_,
                                                                                  transition_time);
        swing_backwards_interpolator_ = std::make_unique<util::LinearInterpolator>(swing_backwards_,
                                                                                   0.0f,
                                                                                   swing_backwards_,
                                                                                   transition_time);
    }

    void LegMotion::update_params(const GaitParams &params) {
        stand_height_interpolator_ = std::make_unique<util::LinearInterpolator>(stand_height_,
                                                                                params.body_height,
                                                                                stand_height_,
                                                                                params.transition_time);
        swing_height_interpolator_ = std::make_unique<util::LinearInterpolator>(swing_height_,
                                                                                params.swing_height,
                                                                                swing_height_,
                                                                                params.transition_time);
        swing_forwards_interpolator_ = std::make_unique<util::LinearInterpolator>(swing_forwards_,
                                                                                  params.swing_max,
                                                                                  swing_forwards_,
                                                                                  params.transition_time);
        swing_backwards_interpolator_ = std::make_unique<util::LinearInterpolator>(swing_backwards_,
                                                                                   params.swing_min,
                                                                                   swing_backwards_,
                                                                                   params.transition_time);
    }

    void LegMotion::timer_tick(float dt) {
        // update interpolators
        if (stand_height_interpolator_) {
            stand_height_interpolator_->update(dt);
            swing_height_interpolator_->update(dt);
            swing_forwards_interpolator_->update(dt);
            swing_backwards_interpolator_->update(dt);
            if (stand_height_interpolator_->finished()) {
                stand_height_interpolator_ = nullptr;
                swing_height_interpolator_ = nullptr;
                swing_forwards_interpolator_ = nullptr;
                swing_backwards_interpolator_ = nullptr;
                if (phase_ == GaitPhase::STAND_TO_GAIT_TRANSITION) {
                    phase_ = GaitPhase::GAITING;
                } else if (phase_ == GaitPhase::GAIT_TO_STAND_TRANSITION) {
                    phase_ = GaitPhase::STANDING;
                }
            }
        }

        // compute position within period
        const float sub_step = gaiting_period_ / 4.0f;
        const float offset = offset_ * sub_step;

        // shift time such that step is at the beginning of the period
        float normalized_time = current_time_ - offset;
        if (normalized_time < 0.0f) {
            normalized_time += gaiting_period_;
        }

        float target_x, target_z,
            target_y = leg_->side() == go2_utils::robot::LegSide::LEFT ? go2_utils::robot::L_1
                                                                       : -go2_utils::robot::L_1;

        if (normalized_time < sub_step) {
            // do the step
            const float x_lift = normalized_time / sub_step;
            target_x = x_lift * swing_backwards_ + (1.0f - x_lift) * swing_forwards_;
            const float z_lift = std::pow(2.0f * x_lift - 1.0f, 2.0f);
            target_z = z_lift * stand_height_ + (1.0f - z_lift) * (stand_height_ - swing_height_);
        } else {
            // push back the legconst float x_lift = normalized_time / sub_step;
            const float x_lift = (normalized_time - sub_step) / (gaiting_period_ - sub_step);
            target_x = x_lift * swing_forwards_ + (1.0f - x_lift) * swing_backwards_;
            target_z = stand_height_;
        }

        auto joints = go2_utils::kinematics::inverse(
            Eigen::Vector3f(target_x, target_y, -target_z),
            leg_->side());

        /*
        RCLCPP_INFO(rclcpp::get_logger("legmotion"), "x: %.4f, y: %.4f, z: %.4f -> %ld solution(s)",
                    target_x, target_y, -target_z, joints.size()); // */

        for (auto &joint_conf : joints) {
            if (joint_conf.y() >= thigh_min && joint_conf.y() <= thigh_max) {
                leg_->command_joint_angles(joint_conf);
                break;
            }
        }

        // increment current phase
        current_time_ += dt;
        if (current_time_ > gaiting_period_) {
            current_time_ -= gaiting_period_;
        }
    }

    bool LegMotion::is_standing() {
        return phase_ == GaitPhase::STANDING;
    }

    bool LegMotion::is_gaiting() {
        return phase_ == GaitPhase::GAITING;
    }
} // namespace basic_motion::controller::states::gait