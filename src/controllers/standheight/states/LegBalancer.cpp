#include "LegBalancer.h"
#include <yaml-cpp/yaml.h>

namespace controllers::standheight::states {
    LegBalancer::LegBalancer(Controller *controller) {
        // Load config from params.yaml
        auto config = YAML::LoadFile("../params.yaml")["leg_balancer"];
        tau_min_ = config["tau_min"].as<float>();
        tau_max_ = config["tau_max"].as<float>();

        // initialize pid controllers
        for (int i = 0; i < 4; ++i) {
            // hips
            hip_pids_[i] = std::make_unique<common::PidController>(
                config["hip"]["kp"].as<float>(),
                config["hip"]["ki"].as<float>(),
                config["hip"]["kd"].as<float>());
            hip_pids_[i]->setpoint(
                config["hip"]["setpoint"].as<float>());

            // thighs
            thigh_pids_[i] = std::make_unique<common::PidController>(
                config["thigh"]["kp"].as<float>(),
                config["thigh"]["ki"].as<float>(),
                config["thigh"]["kd"].as<float>());
            thigh_pids_[i]->setpoint(
                config["thigh"]["setpoint"].as<float>());

            // calfs
            calf_pids_[i] = std::make_unique<common::PidController>(
                config["calf"]["kp"].as<float>(),
                config["calf"]["ki"].as<float>(),
                config["calf"]["kd"].as<float>());
            calf_pids_[i]->setpoint(
                config["calf"]["setpoint"].as<float>());
        }

        // initialize joints
        auto &llc = controller->low_level_control();
        hip_joints_[0] = llc->frontLeft()->hip();
        hip_joints_[1] = llc->frontRight()->hip();
        hip_joints_[2] = llc->backLeft()->hip();
        hip_joints_[3] = llc->backRight()->hip();

        thigh_joints_[0] = llc->frontLeft()->thigh();
        thigh_joints_[1] = llc->frontRight()->thigh();
        thigh_joints_[2] = llc->backLeft()->thigh();
        thigh_joints_[3] = llc->backRight()->thigh();

        calf_joints_[0] = llc->frontLeft()->calf();
        calf_joints_[1] = llc->frontRight()->calf();
        calf_joints_[2] = llc->backLeft()->calf();
        calf_joints_[3] = llc->backRight()->calf();
    }

    void LegBalancer::timer_tick(Controller *controller) {
        for (int i = 0; i < 4; ++i) {
            float current, signal;

            // update hips
            auto &hip = hip_joints_[i];
            current = hip->state().q;
            signal = hip_pids_[i]->control(current, 0.002f /* seconds */); // 2 ms period
            signal = std::clamp(signal, tau_min_, tau_max_);
            hip->mode(1);
            hip->tau(signal);

            // update thighs
            auto &thigh = thigh_joints_[i];
            current = thigh->state().q;
            signal = thigh_pids_[i]->control(current, 0.002f /* seconds */); // 2 ms period
            signal = std::clamp(signal, tau_min_, tau_max_);
            thigh->mode(1);
            thigh->tau(signal);

            // update calfs
            auto &calf = calf_joints_[i];
            current = calf->state().q;
            signal = calf_pids_[i]->control(current, 0.002f /* seconds */); // 2 ms period
            signal = std::clamp(signal, tau_min_, tau_max_);
            calf->mode(1);
            calf->tau(signal);
        }

        controller->low_level_control()->publish();
    }

} // namespace controllers::standheight::states
