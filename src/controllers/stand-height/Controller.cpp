#include "Controller.h"
#include "../../common/Interpolation.h"

namespace controllers::stand_height {
    Controller::Controller(const rclcpp::Node::SharedPtr &node, std::shared_ptr<Config> &config)
        : node_(node), config_(config) {
        // Initialize interfaces
        low_level_control_ = std::make_shared<interface::LowLevelControl>(node);
        motion_switcher_ = std::make_shared<interface::MotionSwitcher>(node);

        // Initialize timer
        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(config->ms_per_tick),
            std::bind(&Controller::timer_tick, this));

        // Compute tick period in seconds
        dt_ = 0.001f * config_->ms_per_tick;
    }

    void Controller::set_done() {
        done_.set_value();
    }

    std::future<void> Controller::done_future() {
        return done_.get_future();
    }

    void Controller::timer_tick() {
        if (startup_ > 0) {
            startup_--;
            // Load first step when startup is has concluded
            if (startup_ == 0) {
                step_iterator_ = config_->steps.begin();
                update_interpolators();
            }
            return;
        }

        for (int i = 0; i < 4; ++i) {
            auto &leg = low_level_control_->leg(i);

            auto &hip = leg->hip();
            auto hip_q = common::interpolate_square_between(
                hip_interpolator[i].pos_curr,
                hip_interpolator[i].from,
                hip_interpolator[i].to);
            hip->q(hip_q);
            hip->mode(1);
            hip->kp(50);
            hip->kd(5);
            hip_interpolator[i].pos_curr += hip_interpolator[i].pos_step;

            auto &thigh = leg->thigh();
            auto thigh_q = common::interpolate_square_between(
                thigh_interpolator[i].pos_curr,
                thigh_interpolator[i].from,
                thigh_interpolator[i].to);
            thigh->q(thigh_q);
            thigh->mode(1);
            thigh->kp(50);
            thigh->kd(5);
            thigh_interpolator[i].pos_curr += thigh_interpolator[i].pos_step;

            auto &calf = leg->calf();
            auto calf_q = common::interpolate_square_between(
                calf_interpolator[i].pos_curr,
                calf_interpolator[i].from,
                calf_interpolator[i].to);
            calf->q(calf_q);
            calf->mode(1);
            calf->kp(50);
            calf->kd(5);
            calf_interpolator[i].pos_curr += calf_interpolator[i].pos_step;

            printf("hip: %.4f\t calf: %.4f\t thigh: %.4f\n", hip_q, thigh_q, calf_q);
        }

        low_level_control_->publish();

        time_remaining_ -= dt_;
        if (time_remaining_ <= 0) {
            step_iterator_++;
            update_interpolators();
        }
    }

    void Controller::update_interpolators() {
        if (step_iterator_ == config_->steps.end()) {
            printf("Reached end of steps.\n");
            set_done();
            return;
        }
        Config::MotionStep &step = *step_iterator_;
        time_remaining_ = step.time + step.keep;
        float pos_step = dt_ / step.time;
        printf("Switching to the next step (time: %.4f, keep: %.4f, step: %.4f).\n",
               step.time,
               step.keep,
               pos_step);

        // initialize cascaded controllers
        for (int i = 0; i < 4; ++i) {
            auto &leg = low_level_control_->leg(i);

            // hip
            hip_interpolator[i].from = std::isnan(hip_interpolator[i].to) ? leg->hip()->state().q : hip_interpolator[i].to;
            hip_interpolator[i].to = (i <= 1 ? step.front : step.back).hip * (i & 1 ? 1.0 : -1.0);
            hip_interpolator[i].pos_step = pos_step;
            hip_interpolator[i].pos_curr = 0.0f;

            // thigh
            thigh_interpolator[i].from = std::isnan(thigh_interpolator[i].to) ? leg->thigh()->state().q : thigh_interpolator[i].to;
            thigh_interpolator[i].to = (i <= 1 ? step.front : step.back).thigh;
            thigh_interpolator[i].pos_step = pos_step;
            thigh_interpolator[i].pos_curr = 0.0f;

            // calf
            calf_interpolator[i].from = std::isnan(calf_interpolator[i].to) ? leg->calf()->state().q : calf_interpolator[i].to;
            calf_interpolator[i].to = (i <= 1 ? step.front : step.back).calf;
            calf_interpolator[i].pos_step = pos_step;
            calf_interpolator[i].pos_curr = 0.0f;
        }
    }

    interface::LowLevelControl::SharedPtr &Controller::low_level_control() {
        return low_level_control_;
    }

} // namespace controllers::stand_height
