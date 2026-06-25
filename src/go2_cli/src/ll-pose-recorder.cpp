#include <chrono>
#include <fstream>
#include <go2_utils/interact/LowLevelControl.h>
#include <go2_utils/kinematics.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>

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

    // create node
    const rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("recorder");
    go2_utils::interact::LowLevelControl llc(node);

    // run the measurements
    unsigned time = 0;
    auto timer = node->create_wall_timer(2ms, [&csv_file, &llc, &time]() {
        time++;
        auto &imu_state = llc.imu_state();
        csv_file
            << time << ','
            << imu_state.rpy[0] << ','
            << imu_state.rpy[1] << ',';
        for (unsigned i = 0; i < 4; i++) {
            auto &leg = llc.leg(i);
            auto joints = Eigen::Vector3f(
                leg->hip()->state().q,
                leg->thigh()->state().q,
                leg->calf()->state().q);
            auto leg_pos = go2_utils::kinematics::forwards(
                joints,
                (i & 1U) ? go2_utils::robot::LegSide::LEFT : go2_utils::robot::LegSide::RIGHT);

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

    rclcpp::spin(node);

    // stop and return result
    rclcpp::shutdown();
    return 0;
}
