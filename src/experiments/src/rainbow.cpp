#include <chrono>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <string>
#include <go2_utils/interact/VuiControl.h>

using namespace std::chrono_literals;

/**
 * Main entry point for stand up tool
 * @param argc Number of program arguments
 * @param argv Program arguments
 */
int main(const int argc, char *argv[]) {

    // initialize rclcpp
    rclcpp::init(argc, argv);

    // create node and executor
    const rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("rainbow");
    go2_utils::interact::VuiControl vui(node);
    rclcpp::spin_some(node);

    // let everything get ready
    std::this_thread::sleep_for(200ms);

    using namespace go2_utils::interact;

    VuiControl::LedColor color_order[] = {VuiControl::RED, VuiControl::YELLOW, VuiControl::GREEN, VuiControl::CYAN, VuiControl::BLUE, VuiControl::PURPLE};
    int current = 0;

    auto future = vui.set_led_color(VuiControl::WHITE);
    rclcpp::spin_until_future_complete(node, future);
    printf("%s\n", future.get() ? "success" : "failure");

    const auto &timer = node->create_wall_timer(std::chrono::milliseconds(300), [&]{
        future = vui.set_led_color(color_order[current]);
        current = (current + 1) % 6;
    });

    rclcpp::spin(node);

    // stop and return result
    rclcpp::shutdown();
    return 0;
}