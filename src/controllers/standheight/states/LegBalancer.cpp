#include "LegBalancer.h"
#include <yaml-cpp/yaml.h>

#include <common/go_constants.h>

namespace controllers::standheight::states {
    LegBalancer::LegBalancer(Controller *controller) {
        // fetch config
        config_ = controller->config();

        // initialize cascaded controllers
        auto &llc = controller->low_level_control();
        for (int i = 0; i < 4; ++i) {
            auto &leg = llc->leg(i);

            // hip
            hip_controllers_[i] = std::make_unique<controller::CascadedJointController>(
                leg->hip(),
                config_->hip);
            hip_controllers_[i]->setpoint(config_->sit_down.hip);

            // thigh
            thigh_controllers_[i] = std::make_unique<controller::CascadedJointController>(
                leg->thigh(),
                config_->thigh);
            thigh_controllers_[i]->setpoint(config_->sit_down.thigh);

            // calf
            calf_controllers_[i] = std::make_unique<controller::CascadedJointController>(
                leg->calf(),
                config_->calf);
            calf_controllers_[i]->setpoint(config_->sit_down.calf);
        }
    }

    void LegBalancer::timer_tick(Controller *controller) {
        float dt = 0.001f * config_->ms_per_tick; // seconds

        for (int i = 0; i < 4; ++i) {
            hip_controllers_[i]->control(dt);
            thigh_controllers_[i]->control(dt);
            calf_controllers_[i]->control(dt);
        }

        controller->low_level_control()->publish();
    }

} // namespace controllers::standheight::states
