#pragma once

#include "states/StateBase.h"
#include <basic_motion/srv/damp.hpp>
#include <basic_motion/srv/gait.hpp>
#include <basic_motion/srv/stand.hpp>
#include <future>
#include <go2_utils/interface/LowLevelControl.h>
#include <go2_utils/robot.h>
#include <rclcpp/rclcpp.hpp>

namespace basic_motion::controller {

    class MotionController {
    private:
        // Assigned rclcpp node
        const rclcpp::Node::SharedPtr node_;

        // Current motion state
        states::StateBase::SharedPtr state_;

        // Control timer
        rclcpp::TimerBase::SharedPtr timer_;
        float timer_period_;

        // Low-level motion controller
        go2_utils::interface::LowLevelControl::SharedPtr llc_;

        // Services
        rclcpp::Service<basic_motion::srv::Damp>::SharedPtr damp_service_;
        rclcpp::Service<basic_motion::srv::Stand>::SharedPtr stand_service_;
        rclcpp::Service<basic_motion::srv::Gait>::SharedPtr gait_service_;

        // Termination future
        std::promise<void> termination_promise_;

        void timer_tick();

        void damp_service(const srv::Damp::Request::SharedPtr req, srv::Damp::Response::SharedPtr res);
        void stand_service(const srv::Stand::Request::SharedPtr req, srv::Stand::Response::SharedPtr res);
        void gait_service(const srv::Gait::Request::SharedPtr req, srv::Gait::Response::SharedPtr res);

    public:
        MotionController(const rclcpp::Node::SharedPtr &node, float timer_period = 0.002f);

        rclcpp::Logger get_logger();

        std::future<void> termination_future();

        void change_state(const states::StateBase::SharedPtr &next);

        go2_utils::interface::LowLevelControl::SharedPtr &low_level_control();
    };
} // namespace basic_motion::controller
