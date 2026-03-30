#include "LegBalancer.h"
#include <yaml-cpp/yaml.h>

#include <common/go_constants.h>

namespace controllers::standheight::states {
    LegBalancer::LegBalancer(Controller *controller) {
        // Load config from params.yaml
        auto config = YAML::LoadFile("../params.yaml")["leg_balancer"];
        auto hip_config = config["hip"];
        auto thigh_config = config["thigh"];
        auto calf_config = config["calf"];

        controller::CascadedJointController::Config cjc_config_hip{
            .pos_kp = hip_config["position"]["kp"].as<float>(),
            .pos_ki = hip_config["position"]["ki"].as<float>(),
            .pos_kd = hip_config["position"]["kd"].as<float>(),
            .velo_kp = hip_config["velocity"]["kp"].as<float>(),
            .velo_ki = hip_config["velocity"]["ki"].as<float>(),
            .velo_kd = hip_config["velocity"]["kd"].as<float>(),
            .tau_min = config["tau_min"].as<float>(),
            .tau_max = config["tau_max"].as<float>(),
        };

        controller::CascadedJointController::Config cjc_config_thigh{
            .pos_kp = thigh_config["position"]["kp"].as<float>(),
            .pos_ki = thigh_config["position"]["ki"].as<float>(),
            .pos_kd = thigh_config["position"]["kd"].as<float>(),
            .velo_kp = thigh_config["velocity"]["kp"].as<float>(),
            .velo_ki = thigh_config["velocity"]["ki"].as<float>(),
            .velo_kd = thigh_config["velocity"]["kd"].as<float>(),
            .tau_min = config["tau_min"].as<float>(),
            .tau_max = config["tau_max"].as<float>(),
        };

        controller::CascadedJointController::Config cjc_config_calf{
            .pos_kp = calf_config["position"]["kp"].as<float>(),
            .pos_ki = calf_config["position"]["ki"].as<float>(),
            .pos_kd = calf_config["position"]["kd"].as<float>(),
            .velo_kp = calf_config["velocity"]["kp"].as<float>(),
            .velo_ki = calf_config["velocity"]["ki"].as<float>(),
            .velo_kd = calf_config["velocity"]["kd"].as<float>(),
            .tau_min = config["tau_min"].as<float>(),
            .tau_max = config["tau_max"].as<float>(),
        };

        const float setpoint_hip = hip_config["setpoint"].as<float>();
        const float setpoint_thigh = thigh_config["setpoint"].as<float>();
        const float setpoint_calf = calf_config["setpoint"].as<float>();

        // initialize cascaded controllers
        auto &llc = controller->low_level_control();
        for (int i = 0; i < 4; ++i) {
            auto &leg = llc->leg(i);

            // hip
            hip_controllers_[i] = std::make_unique<controller::CascadedJointController>(
                leg->hip(),
                setpoint_hip,
                cjc_config_hip
            );

            // thigh
            thigh_controllers_[i] = std::make_unique<controller::CascadedJointController>(
                leg->thigh(),
                setpoint_thigh,
                cjc_config_thigh
            );
            
            // calf
            calf_controllers_[i] = std::make_unique<controller::CascadedJointController>(
                leg->calf(),
                setpoint_calf,
                cjc_config_calf
            );
        }
    }

    void LegBalancer::timer_tick(Controller *controller) {
        for (int i = 0; i < 4; ++i) {
            hip_controllers_[i]->control(0.02f /* seconds */);
            //thigh_controllers_[i]->control(0.02f /* seconds */);
            //calf_controllers_[i]->control(0.02f /* seconds */);
        }

        // hip_controllers_[0]->control(0.02);
        // thigh_controllers_[0]->control(0.02);
        calf_controllers_[common::constants::BACK_LEFT_INDEX]->control(0.02);

        controller->low_level_control()->backLeft()->calf()->tau(0.0);
        controller->low_level_control()->publish();
    }

} // namespace controllers::standheight::states
