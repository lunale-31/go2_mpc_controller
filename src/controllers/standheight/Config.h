#pragma once

#include <string>
#include <memory>
#include <yaml-cpp/yaml.h>

namespace controllers::standheight
{
    struct Config
    {
        struct Joint
        {
            struct PidGains
            {
                float kp, ki, kd;
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
        
        struct PoseSetpoint {
            float hip, thigh, calf;
        };

        Joint hip, thigh, calf;

        // Control period in milliseconds
        unsigned ms_per_tick;

        PoseSetpoint sit_down;

        static std::shared_ptr<Config> load(const std::string &filename) {
            auto config = std::make_shared<Config>();

            // Read YAML file
            const auto config_file = YAML::LoadFile(filename)["stand_height"];

            // Read Joint
            static auto read_joint = [](const YAML::Node &node, Config::Joint &joint) {
                // Read PidGains
                static auto read_pid_gains = [](const YAML::Node &node, Config::Joint::PidGains &pid_gains) {
                    pid_gains.kp = node["kp"].as<float>();
                    pid_gains.ki = node["ki"].as<float>();
                    pid_gains.kd = node["kd"].as<float>();
                };

                read_pid_gains(node["position"], joint.pos_gains);
                read_pid_gains(node["velocity"], joint.velo_gains);
                joint.outer_factor = node["outer_factor"].as<unsigned>();
                joint.tau_min = node["tau_min"].as<float>();
                joint.tau_max = node["tau_max"].as<float>();
                joint.motion_speed = node["motion_speed"].as<float>();
            };

            // Read PoseSetpoint
            static auto read_pose_setpoint = [](const YAML::Node &node, Config::PoseSetpoint &pose_sp) {
                pose_sp.hip = node["hip"].as<float>();
                pose_sp.thigh = node["thigh"].as<float>();
                pose_sp.calf = node["calf"].as<float>();
            };

            // Read data
            read_joint(config_file["hip"], config->hip);
            read_joint(config_file["thigh"], config->thigh);
            read_joint(config_file["calf"], config->calf);
            read_pose_setpoint(config_file["sit_down"], config->sit_down);
            config->ms_per_tick = config_file["ms_per_tick"].as<unsigned>();

            return config;
        }
    };
} // namespace controllers::standheight
