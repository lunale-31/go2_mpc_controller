#include "../../interface/LowLevelControl.h"
#include "../../interface/MotionSwitcher.h"
#include "Config.h"
#include <future>
#include <rclcpp/node.hpp>

namespace controllers::stand_height {

    class Controller {
    public:
        class State {
        public:
            virtual void timer_tick(Controller *controller) = 0;
        };

        Controller(const rclcpp::Node::SharedPtr &node, std::shared_ptr<Config> &config);
        void switch_state(const std::shared_ptr<State> &next);

        // rclcpp::Node::SharedPtr &node();
        interface::LowLevelControl::SharedPtr &low_level_control();
        // interface::MotionSwitcher::SharedPtr &motion_switcher();
        // std::shared_ptr<Config> &config();

        void set_done();
        std::future<void> done_future();

    private:
        void timer_tick();

        void update_interpolators();

        rclcpp::Node::SharedPtr node_;
        rclcpp::TimerBase::SharedPtr timer_;
        interface::LowLevelControl::SharedPtr low_level_control_;
        interface::MotionSwitcher::SharedPtr motion_switcher_;
        float dt_;

        // Config
        std::shared_ptr<Config> config_;

        // Motion state
        std::list<Config::MotionStep>::iterator step_iterator_;

        struct Interpolator {
            float from = NAN, to = NAN;
            float pos_curr = 0.0f, pos_step;
        };
        float time_remaining_;

        unsigned startup_ = 500;

        Interpolator hip_interpolator[4], thigh_interpolator[4], calf_interpolator[4];

        // Whether the precess is done
        std::promise<void> done_;
    };

} // namespace controllers::stand_height
