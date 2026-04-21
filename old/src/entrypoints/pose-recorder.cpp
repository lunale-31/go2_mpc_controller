#include <chrono>
#include <fstream>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>

#include "../common/Kinematics.h"
#include "../interface/LowLevelControl.h"

static const std::string REQUEST_TOPIC = "/lowcmd";

using namespace std::chrono_literals;

/**
 * Main entry point for sit down tool
 * @param argc Number of program arguments
 * @param argv Program arguments
 */
int main(const int argc, char* argv[]) {
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
    std::string fields[] = {"time", "roll", "pitch"};
    for (auto &field : fields) {
        csv_file << field << ',';
    }
    std::string leg_prefixes[] = {"FR_", "FL_", "BR_", "BL_"};
    std::string coordinates[] = {"x", "y", "z", "hip", "thigh", "calf"};
    for (auto &leg_prefix : leg_prefixes) {
        for (auto &coord : coordinates) {
            csv_file << leg_prefix << coord << ",";
        }
    }
    csv_file << std::endl;

    rclcpp::init(argc, argv);

    // create node and executor
    rclcpp::executors::SingleThreadedExecutor executor;
    const rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("recorder");
    interface::LowLevelControl llc(node);
    executor.add_node(node);

    // let everything get ready
    std::this_thread::sleep_for(200ms);

    // run the measurements
    unsigned time = 0;
    auto timer = node->create_wall_timer(2ms, [&csv_file, &llc, &time]() {
        time++;
        auto &imu_state = llc.imu_state();
        csv_file
            << time << ','
            << imu_state->roll << ','
            << imu_state->pitch << ',';
        for (unsigned i = 0; i < 4; i++) {
            auto &leg = llc.leg(i);
            auto joints = Eigen::Vector3f(
                leg->hip()->state().q,
                leg->thigh()->state().q,
                leg->calf()->state().q);
            auto leg_pos = common::forwards_kinematics(joints, (i & 1U) ? common::LEFT : common::RIGHT);

            csv_file
                << leg_pos->x() << ','
                << leg_pos->y() << ','
                << leg_pos->z() << ','
                << leg->hip()->state().q << ','
                << leg->thigh()->state().q << ','
                << leg->calf()->state().q << ',';
        }
        csv_file << std::endl;
    });

    executor.spin();

    // stop and return result
    rclcpp::shutdown();
    return 0;
}
