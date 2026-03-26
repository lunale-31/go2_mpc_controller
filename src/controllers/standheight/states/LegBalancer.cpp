#include "LegBalancer.h"
#include <yaml-cpp/yaml.h>

namespace controllers::standheight::states {
    LegBalancer::LegBalancer(Controller *controller) {
        // Prevent warning
        (void)controller;

        // Load config from params.yaml
        YAML::Node config = YAML::LoadFile("../params.yaml");
        max_q = config["max_q"].as<float>();
        t_max_q = config["t_max_q"].as<float>();
        min_q = config["min_q"].as<float>();
        t_min_q = config["t_min_q"].as<float>();
        dq_pos = config["dq_pos"].as<float>();
        dq_neg = config["dq_neg"].as<float>();
        kp = config["kp"].as<float>();
        kd = config["kd"].as<float>();
        tau = config["tau"].as<float>();
    }

    void LegBalancer::timer_tick(Controller *controller) {
        t_ += dt_;
        if (t_ >= 2 * M_PI)
            t_ -= 2 * M_PI;

        float pos = (cos(t_) + 1.0) / 2.0; // yields 0.0 <= pos <= 1.0

        float target_q = (1.0 - pos) * min_q + pos * max_q;

        // old code
        auto &calf = controller->low_level_control()->backRight().calf();
        calf.mode(0x1);
        calf.q(target_q);
        calf.dq(!move_forwards_ ? dq_pos : dq_neg);
        calf.kp(kp);
        calf.kd(kd);
        calf.tau(tau);
        controller->low_level_control()->publish();
    }

} // namespace controllers::standheight::states
