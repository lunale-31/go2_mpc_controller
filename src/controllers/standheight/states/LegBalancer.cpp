#include "LegBalancer.h"
#include "../../../common/Interpolation.h"
#include <common/go_constants.h>
#include <yaml-cpp/yaml.h>

namespace controllers::standheight::states {
    LegBalancer::LegBalancer(Controller *controller) {
        // fetch config
        config_ = controller->config();

        // tick period in seconds
        float dt = 0.001f * config_->ms_per_tick;

        // initialize cascaded controllers
        auto &llc = controller->low_level_control();
        for (int i = 0; i < 4; ++i) {
            auto &leg = llc->leg(i);

            // hip
            hip_controllers_[i] = std::make_unique<controller::CascadedJointController>(
                leg->hip(),
                config_->hip);

            hip_interpolator[i].from = leg->hip()->state().q;
            hip_interpolator[i].to = config_->sit_down.hip * (i & 1 ? 1.0 : -1.0);
            hip_interpolator[i].pos_step = config_->hip.motion_speed * dt / std::abs(hip_interpolator[i].to - hip_interpolator[i].from);

            printf(
                "hip %d: from %.4f to %.4f in %d steps (%.4f seconds)\n",
                i, hip_interpolator[i].from, hip_interpolator[i].to,
                static_cast<int>(std::ceil(1.0f / hip_interpolator[i].pos_step)),
                dt / (hip_interpolator[i].pos_step));

            // thigh
            thigh_controllers_[i] = std::make_unique<controller::CascadedJointController>(
                leg->thigh(),
                config_->thigh);
            thigh_controllers_[i]->setpoint(config_->sit_down.thigh);

            thigh_interpolator[i].from = leg->thigh()->state().q;
            thigh_interpolator[i].to = config_->sit_down.thigh;
            thigh_interpolator[i].pos_step = config_->thigh.motion_speed * dt / std::abs(thigh_interpolator[i].to - thigh_interpolator[i].from);

            printf(
                "thigh %d: from %.4f to %.4f in %d steps (%.4f seconds)\n",
                i, thigh_interpolator[i].from, thigh_interpolator[i].to,
                static_cast<int>(std::ceil(1.0f / thigh_interpolator[i].pos_step)),
                dt / (thigh_interpolator[i].pos_step));

            // calf
            calf_controllers_[i] = std::make_unique<controller::CascadedJointController>(
                leg->calf(),
                config_->calf);
            calf_controllers_[i]->setpoint(config_->sit_down.calf);

            calf_interpolator[i].from = leg->calf()->state().q;
            calf_interpolator[i].to = config_->sit_down.calf;
            calf_interpolator[i].pos_step = config_->calf.motion_speed * dt / std::abs(calf_interpolator[i].to - calf_interpolator[i].from);

            printf(
                "calf %d: from %.4f to %.4f in %d steps (%.4f seconds)\n",
                i, calf_interpolator[i].from, calf_interpolator[i].to,
                static_cast<int>(std::ceil(1.0f / calf_interpolator[i].pos_step)),
                dt / calf_interpolator[i].pos_step);
        }
    }

    void LegBalancer::timer_tick(Controller *controller) {
        float dt = 0.001f * config_->ms_per_tick; // seconds

        for (int i = 0; i < 4; ++i) {
            hip_controllers_[i]->setpoint(common::interpolate_square_between(
                hip_interpolator[i].pos_curr,
                hip_interpolator[i].from,
                hip_interpolator[i].to));
            hip_controllers_[i]->control(dt);
            hip_interpolator[i].pos_curr += hip_interpolator[i].pos_step;

            thigh_controllers_[i]->setpoint(common::interpolate_square_between(
                thigh_interpolator[i].pos_curr,
                thigh_interpolator[i].from,
                thigh_interpolator[i].to));
            thigh_controllers_[i]->control(dt);
            thigh_interpolator[i].pos_curr += thigh_interpolator[i].pos_step;

            printf(
                "thigh %d: q_r = %.4f, q = %.4f, u = %.4f\n",
                i,
                thigh_controllers_[i]->setpoint(),
                thigh_controllers_[i]->current(),
                thigh_controllers_[i]->signal());

            calf_controllers_[i]->setpoint(common::interpolate_square_between(
                calf_interpolator[i].pos_curr,
                calf_interpolator[i].from,
                calf_interpolator[i].to));
            calf_controllers_[i]->control(dt);
            calf_interpolator[i].pos_curr += calf_interpolator[i].pos_step;
        }

        controller->low_level_control()->publish();
    }

} // namespace controllers::standheight::states
