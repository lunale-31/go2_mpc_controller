#include <go2_utils/interact/LowLevelControl.h>
#include <go2_utils/kinematics.h>
#include <rclcpp/rclcpp.hpp>
#include <cstdint>

using namespace go2_utils::interact;

std::shared_ptr<rclcpp::Node> node;
std::shared_ptr<LowLevelControl> llc;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>("foot_forces");
    llc = std::make_shared<LowLevelControl>(node);

    const auto &logger = node->get_logger();

    // Spin until there is data
    RCLCPP_INFO(logger, "Waiting to receive lowlevel data...");
    while (!llc->was_state_received()) {
        rclcpp::spin_some(node);
    }
    RCLCPP_INFO(logger, "Received data on both channels.");

    auto timer = node->create_wall_timer(std::chrono::milliseconds(20), [&logger] {
        uint16_t fl_force = llc->frontLeft()->foot_force();
        uint16_t fr_force = llc->frontRight()->foot_force();
        uint16_t bl_force = llc->backLeft()->foot_force();
        uint16_t br_force = llc->backRight()->foot_force();

        RCLCPP_INFO(logger, "Got foot forces. FL: %4d FR: %4d BL: %4d BR: %4d.", fl_force, fr_force, bl_force, br_force);
    });

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
