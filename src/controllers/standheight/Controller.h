#pragma once
#include "../../interface/LowLevelControl.h"
#include "../../interface/MotionSwitcher.h"
#include "Config.h"
#include <rclcpp/node.hpp>

namespace controllers::standheight {

    class Controller {
    public:
        class State {
        public:
            virtual void timer_tick(Controller *controller) = 0;
        };

        Controller(const rclcpp::Node::SharedPtr &node, std::shared_ptr<Config> &config);
        void switch_state(const std::shared_ptr<State> &next);

        rclcpp::Node::SharedPtr &node();
        interface::LowLevelControl::SharedPtr &low_level_control();
        interface::MotionSwitcher::SharedPtr &motion_switcher();
        std::shared_ptr<Config> &config();

    private:
        void timer_tick();

        rclcpp::Node::SharedPtr node_;
        rclcpp::TimerBase::SharedPtr timer_;
        interface::LowLevelControl::SharedPtr low_level_control_;
        interface::MotionSwitcher::SharedPtr motion_switcher_;

        // Config
        std::shared_ptr<Config> config_;

        // Process state (state machine)
        std::shared_ptr<State> state_;
    };

} // namespace controllers::standheight
