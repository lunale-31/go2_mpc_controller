#include "MotionSwitcherCheck.h"
#include "LegBalancer.h"
#include <chrono>

using namespace std::chrono_literals;

namespace controllers::lowlevel::states {
    MotionSwitcherCheck::MotionSwitcherCheck(Controller *controller) {
        RCLCPP_INFO(
            controller->node()->get_logger(),
            "Checking that motion was actually turned off.");
    }

    void MotionSwitcherCheck::timer_tick(Controller *controller) {
        if (!request_sent_) {
            motion_switch_check_result_ = controller->motion_switcher()->get_silent();
            request_sent_ = true;
            return;
        }

        RCLCPP_INFO(controller->node()->get_logger(), "Waiting...");
        if (motion_switch_check_result_.wait_for(100ns) == std::future_status::ready) {
            bool silent = motion_switch_check_result_.get();
            RCLCPP_INFO(
                controller->node()->get_logger(),
                "Received response, turning off motion %s.",
                silent ? "succeeded" : "failed");
            if (silent) {
                controller->switch_state(std::make_shared<LegBalancer>(controller));
            } else {
                controller->switch_state(nullptr);
            }
        }
    }
} // namespace controllers::lowlevel
