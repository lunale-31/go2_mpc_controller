#include "MotionSwitcherSet.h"
#include "MotionSwitcherCheck.h"
#include <chrono>

using namespace std::chrono_literals;

namespace controllers::standheight::states {
    MotionSwitcherSet::MotionSwitcherSet(Controller *controller) {
        RCLCPP_INFO(
            controller->node()->get_logger(),
            "Requesting motion to be turned off.");
        motion_switch_result_ = controller->motion_switcher()->set_silent(true);
    }

    void MotionSwitcherSet::timer_tick(Controller *controller) {
        RCLCPP_INFO(controller->node()->get_logger(), "Waiting...");
        if (motion_switch_result_.wait_for(100ns) == std::future_status::ready) {
            bool success = motion_switch_result_.get();
            RCLCPP_INFO(
                controller->node()->get_logger(),
                "Received response, turning off motion %s.",
                success ? "succeeded" : "failed");
            if (success) {
                controller->switch_state(std::make_shared<MotionSwitcherCheck>(controller));
            } else {
                controller->switch_state(nullptr);
            }
        }
    }
} // namespace controllers::standheight::states
