#pragma once

#include <memory>
#include <string>
#include <list>
#include <yaml-cpp/yaml.h>

namespace controllers::stand_height {
    struct Config {
        struct Joint {
            struct PidGains {
                float K, Ti, Td, N, Beta, Tr;

                // Smoothing time constant
                float Tf;
            };

            // Gains for the position (i.e. outer) PID controller
            PidGains pos_gains;

            // Gains for the velocity (i.e. inner) PID controller
            PidGains velo_gains;

            // Relative execution rate of the inner to the outer controller
            unsigned outer_factor;

            // Torque limits
            float tau_min, tau_max;

            // Desired motion speed (in rad/s)
            float motion_speed;
        };

        struct JointSetpoint {
            float hip, thigh, calf;
        };

        struct MotionStep {
            JointSetpoint front, back;
            float time;
            float keep = 0.0f;
        };

        Joint hip, thigh, calf;

        // Control period in milliseconds
        unsigned ms_per_tick;

        std::list<MotionStep> steps;

        static std::shared_ptr<Config> load(const std::string &filename) {
            auto config = std::make_shared<Config>();

            // Read YAML file
            const auto config_file = YAML::LoadFile(filename)["stand_height"];

            // Read Joint
            static auto read_joint = [](const YAML::Node &node, Config::Joint &joint) {
                // Read PidGains
                static auto read_pid_gains = [](const YAML::Node &node, Config::Joint::PidGains &pid_gains) {
                    pid_gains.K = node["K"].as<float>();
                    pid_gains.Ti = node["Ti"].as<float>();
                    pid_gains.Td = node["Td"].as<float>();
                    pid_gains.N = node["N"].as<float>();
                    pid_gains.Beta = node["Beta"].as<float>();
                    if (!node["Tr"].IsDefined() || node["Tr"].IsNull()) {
                        pid_gains.Tr = sqrtf(pid_gains.Ti * pid_gains.Td);
                    } else {
                        pid_gains.Tr = node["Tr"].as<float>();
                    }
                    pid_gains.Tf = node["Tf"].as<float>();
                };

                read_pid_gains(node["position"], joint.pos_gains);
                read_pid_gains(node["velocity"], joint.velo_gains);
                joint.outer_factor = node["outer_factor"].as<unsigned>();
                joint.tau_min = node["tau_min"].as<float>();
                joint.tau_max = node["tau_max"].as<float>();
                joint.motion_speed = node["motion_speed"].as<float>();
            };

            static auto read_motion_steps = [](const YAML::Node &node, std::list<Config::MotionStep> &steps) {
                // Read pose setpoint
                static auto read_pose_setpoint = [](const YAML::Node &node, Config::JointSetpoint &pose_sp) {
                    pose_sp.hip = node["hip"].as<float>();
                    pose_sp.thigh = node["thigh"].as<float>();
                    pose_sp.calf = node["calf"].as<float>();
                };

                for (auto &step_node : node) {
                    auto &step = steps.emplace_back();
                    if (step_node["all"].IsDefined()) {
                        read_pose_setpoint(step_node["all"], step.front);
                        read_pose_setpoint(step_node["all"], step.back);
                    } else {
                        read_pose_setpoint(step_node["front"], step.front);
                        read_pose_setpoint(step_node["back"], step.back);
                    }
                    step.time = step_node["time"].as<float>();
                    if (step_node["keep"].IsDefined()) {
                        step.keep = step_node["keep"].as<float>();
                    }
                }
            };

            // Read data
            read_joint(config_file["hip"], config->hip);
            read_joint(config_file["thigh"], config->thigh);
            read_joint(config_file["calf"], config->calf);
            read_motion_steps(config_file["steps"], config->steps);
            config->ms_per_tick = config_file["ms_per_tick"].as<unsigned>();

            return config;
        }
    };
} // namespace controllers::standheight
