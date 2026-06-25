#include <cstdio>
#include <fstream>
#include <future>
#include <go2_utils/interact/HighLevelControl.h>
#include <go2_utils/interact/HighLevelState.h>
#include <go2_utils/interact/LowLevelControl.h>
#include <go2_utils/kinematics.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <unitree_go/msg/low_cmd.hpp>

using namespace go2_utils::interact;

std::shared_ptr<rclcpp::Node> node;
std::shared_ptr<HighLevelControl> hlc;
std::shared_ptr<HighLevelState> hls;
std::shared_ptr<LowLevelControl> llc;

void await_data() {
    // Spin until there is data
    while (!hls->was_state_received() || !llc->was_state_received()) {
        rclcpp::spin_some(node);
    }
}

void walk_and_stop(float target_distance, float xVelo, float yVelo) {
    bool done = false;

    // Fulfill promise once distance was reached
    std::array<float, 3> start_pos = hls->position();
    auto check_timer = node->create_wall_timer(std::chrono::milliseconds(2), [&]() {
        std::array<float, 3> &current_pos = hls->position();
        float current_distance = 0.0f;
        for (int i = 0; i < 3; i++) {
            current_distance += std::pow(start_pos[i] - current_pos[i], 2.0);
        }
        if (current_distance >= target_distance) {
            done = true;
        }
    });

    // walk distance
    while (!done) {
        rclcpp::spin_until_future_complete(node, hlc->move(xVelo, yVelo, 0.0f));
    }

    // stop movement
    rclcpp::spin_until_future_complete(node, hlc->move(0.0f, 0.0f, 0.0f));
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>("gait_logger");
    hlc = std::make_shared<HighLevelControl>(node);
    hls = std::make_shared<HighLevelState>(node);
    llc = std::make_shared<LowLevelControl>(node);

    const auto &logger = node->get_logger();

    // Wait to get high and lowlevel data
    RCLCPP_INFO(logger, "Waiting to receive high- and lowlevel data...");
    await_data();
    RCLCPP_INFO(logger, "Received data on both channels.");

    // Prepare data collector
    // TODO: Time, Commands (kp, kd, tau, q, dq), Joint states (tau_est, q, dq), Foot positions, Robot position
    std::string legs[] = {"fr", "fl", "br", "bl"};
    std::string joints[] = {"hip", "thigh", "calf"};
    std::string joint_params[] = {"kp", "kd", "tau", "q_r", "dq_r", "tau_est", "q", "dq"};
    std::string coords[] = {"x", "y", "z"};

    std::ofstream dumpfile;
    dumpfile.open("gait-dump.csv");
    dumpfile << "time,";
    for (const auto &leg : legs) {
        for (const auto &joint : joints) {
            for (const auto &param : joint_params) {
                dumpfile << leg << "_" << joint << "_" << param << ",";
            }
        }
    }
    for (const auto &leg : legs) {
        for (const auto &coord : coords) {
            dumpfile << leg << "_foot_" << coord << ",";
        }
    }
    for (const auto &coord : coords) {
        dumpfile << "robot_" << coord << ",";
    }

    dumpfile << std::endl;

    unsigned long tick = 0;

    auto collector = node->create_subscription<unitree_go::msg::LowCmd>(
        "/lowcmd", 10, [&](const unitree_go::msg::LowCmd &cmd) {
            dumpfile << tick++ << ",";
            for (int i = 0; i < 12; i++) {
                auto &joint = llc->joint(i);
                dumpfile << cmd.motor_cmd[i].kp << ","
                         << cmd.motor_cmd[i].kd << ","
                         << cmd.motor_cmd[i].tau << ","
                         << cmd.motor_cmd[i].q << ","
                         << cmd.motor_cmd[i].dq << ","
                         << joint->state().tau_est << ","
                         << joint->state().q << ","
                         << joint->state().dq << ",";
            }
            for (int i = 0; i < 4; i++) {
                const auto &leg = llc->leg(i);
                const auto &foot_pos = leg->foot_position();
                dumpfile << foot_pos.x() << ","
                         << foot_pos.y() << ","
                         << foot_pos.z() << ",";
            }
            const auto &pos = hls->position();
            dumpfile << pos[0] << ","
                     << pos[1] << ","
                     << pos[2] << ","
                     << std::endl;
        });

    // Walk forward for 10 meters
    RCLCPP_INFO(logger, "Started fast forwards walk at tick %lu.", tick);
    walk_and_stop(10.0f /* meters */, 1.0f, 0.0f);
    RCLCPP_INFO(logger, "Stopped fast forwards walk at tick %lu.", tick);

    // Walk backwards for 10 meters
    RCLCPP_INFO(logger, "Started fast backwards walk at tick %lu.", tick);
    walk_and_stop(10.0f /* meters */, -1.0f, 0.0f);
    RCLCPP_INFO(logger, "Stopped fast backwards walk at tick %lu.", tick);

    // Walk forward for 10 meters
    RCLCPP_INFO(logger, "Started medium forwards walk at tick %lu.", tick);
    walk_and_stop(10.0f /* meters */, 0.6f, 0.0f);
    RCLCPP_INFO(logger, "Stopped medium forwards walk at tick %lu.", tick);

    // Walk backwards for 10 meters
    RCLCPP_INFO(logger, "Started medium backwards walk at tick %lu.", tick);
    walk_and_stop(10.0f /* meters */, -0.6f, 0.0f);
    RCLCPP_INFO(logger, "Stopped medium backwards walk at tick %lu.", tick);

    // Walk forward for 10 meters
    RCLCPP_INFO(logger, "Started slow forwards walk at tick %lu.", tick);
    walk_and_stop(10.0f /* meters */, 0.3f, 0.0f);
    RCLCPP_INFO(logger, "Stopped slow forwards walk at tick %lu.", tick);

    // Walk backwards for 10 meters
    RCLCPP_INFO(logger, "Started slow backwards walk at tick %lu.", tick);
    walk_and_stop(10.0f /* meters */, -0.3f, 0.0f);
    RCLCPP_INFO(logger, "Stopped slow backwards walk at tick %lu.", tick);

    rclcpp::shutdown();

    return 0;
}
