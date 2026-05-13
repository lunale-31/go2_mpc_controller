#include <chrono>
#include <fstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <go2_utils/interact/LowLevelControl.h>

static const std::string REQUEST_TOPIC = "/lowcmd";

using namespace std::chrono_literals;

/**
 * Main entry point for sit down tool
 * @param argc Number of program arguments
 * @param argv Program arguments
 */
int main(const int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <output.csv|->" << std::endl;
        return -1;
    }

    // Try to open output file
    std::ofstream csv_file(std::string(argv[1]) == "-" ? "/dev/stdout" : argv[1]);
    if (!csv_file.is_open()) {
        std::cerr << "Could not open output file " << std::quoted(argv[1]) << "." << std::endl;
        return -2;
    }

    // Create CSV header
    csv_file << "time,";
    std::string leg_prefixes[] = {"FR_", "FL_", "BR_", "BL_"};
    std::string joint_prefixes[] = {"HIP_", "THIGH_", "CALF_"};
    std::string joint_params[] = {"mode", "kp", "kd", "q_r", "dq_r", "tau", "q", "dq"};
    for (auto &leg_prefix : leg_prefixes) {
        for (auto &joint_prefix : joint_prefixes) {
            for (auto &joint_param : joint_params) {
                csv_file << leg_prefix << joint_prefix << joint_param << ",";
            }
        }
    }
    csv_file << std::endl;

    rclcpp::init(argc, argv);

    // create node and executor
    const rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("recorder");
    go2_utils::interact::LowLevelControl llc(node);

    // prepare recorder
    unsigned time = 0;
    unsigned last_change = 0;
    unitree_go::msg::LowCmd cmd;
    auto subscription = node->create_subscription<unitree_go::msg::LowCmd>(
        REQUEST_TOPIC, 10,
        [&cmd, &last_change](const unitree_go::msg::LowCmd &new_cmd) {
            bool changed = cmd.crc != new_cmd.crc;
            if (changed) {
                cmd = new_cmd;
                last_change = 0;
            } else {
                last_change++;
            }
        });

    // prepare csv writer
    auto timer = node->create_wall_timer(2ms, [&cmd, &csv_file, &llc, &time, &last_change]() {
        time++;
        if (last_change > 20) {
            return;
        }
        csv_file << time << ',';
        for (unsigned i = 0; i < 12; ++i) {
            auto &motor = cmd.motor_cmd[i];
            auto &motor_state = llc.joint(i)->state();
            csv_file
                << static_cast<int>(motor.mode) << ','
                << motor.kp << ','
                << motor.kd << ','
                << motor.q << ','
                << motor.dq << ','
                << motor.tau << ','
                << motor_state.q << ','
                << motor_state.dq << ',';
        }
        csv_file << std::endl;
    });

    // run the measurements
    rclcpp::spin(node);

    // stop and return result
    rclcpp::shutdown();
    return 0;
}
