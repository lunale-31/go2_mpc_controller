#include "LegBalancer.h"
#include <yaml-cpp/yaml.h>

namespace controllers::standheight::states {
    LegBalancer::LegBalancer(Controller *controller) {
        // Prevent warning
        (void)controller;

        // Load config from params.yaml
        auto config = YAML::LoadFile("../params.yaml");
        auto pid_config = config["leg_balancer"];
        pid_ = std::make_unique<common::PidController>(
            pid_config["kp"].as<float>(),
            pid_config["ki"].as<float>(),
            pid_config["kd"].as<float>()
        );
        setpoint_ = pid_config["setpoint"].as<float>();
        tau_min_ = pid_config["tau_min"].as<float>();
        tau_max_ = pid_config["tau_max"].as<float>();
    }

    void LegBalancer::timer_tick(Controller *controller) {
        auto &calf = controller->low_level_control()->backRight().calf();

        float current = calf.state().q;
        float signal = pid_->control(setpoint_, current, 0.02f);

        signal = std::clamp(signal, tau_min_, tau_max_);

        calf.mode(0x1);
        calf.q(0.0);
        calf.dq(0.0);
        calf.kp(0.0);
        calf.kd(0.0);
        calf.tau(signal);
        controller->low_level_control()->publish();
    }

} // namespace controllers::standheight::states
