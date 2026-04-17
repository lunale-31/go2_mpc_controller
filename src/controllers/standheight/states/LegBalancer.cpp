#include "LegBalancer.h"
#include "../../../common/Interpolation.h"
#include <common/go_constants.h>
#include <yaml-cpp/yaml.h>

namespace controllers::standheight::states {
    LegBalancer::LegBalancer(Controller *controller) {
        // fetch config
        config_ = controller->config();

        // tick period in seconds
        dt_ = 0.001f * config_->ms_per_tick;

        // initialize controllers
        for (int i = 0; i < 4; ++i) {
            auto &leg = controller->low_level_control()->leg(i);

            // hip
            hip_controllers_[i] = std::make_unique<controller::CascadedJointController>(
                leg->hip(),
                config_->hip);

            // thigh
            thigh_controllers_[i] = std::make_unique<controller::CascadedJointController>(
                leg->thigh(),
                config_->thigh);

            // calf
            calf_controllers_[i] = std::make_unique<controller::CascadedJointController>(
                leg->calf(),
                config_->calf);
        }

        // initialize iterators
        step_iterator_ = config_->steps.begin();
        update_interpolators(controller);
    }

    void LegBalancer::timer_tick(Controller *controller) {
        // Emergency stop
        for (int i = 0; i < 4; ++i) {
            auto &joint = controller->low_level_control()->joint(i);
            if (abs(joint->state().dq) >= 5.0f) {
                printf("Joint %d has a velocity of %.4f. Emergency stop!\n", i, joint->state().dq);
                /*controller->set_done();
                return;*/
            }
        }

        for (int i = 0; i < 4; ++i) {
            hip_controllers_[i]->setpoint(common::interpolate_square_between(
                hip_interpolator[i].pos_curr,
                hip_interpolator[i].from,
                hip_interpolator[i].to));
            hip_controllers_[i]->control(dt_);
            hip_interpolator[i].pos_curr += hip_interpolator[i].pos_step;

            thigh_controllers_[i]->setpoint(common::interpolate_square_between(
                thigh_interpolator[i].pos_curr,
                thigh_interpolator[i].from,
                thigh_interpolator[i].to));
            thigh_controllers_[i]->control(dt_);
            thigh_interpolator[i].pos_curr += thigh_interpolator[i].pos_step;

            calf_controllers_[i]->setpoint(common::interpolate_square_between(
                calf_interpolator[i].pos_curr,
                calf_interpolator[i].from,
                calf_interpolator[i].to));
            calf_controllers_[i]->control(dt_);
            calf_interpolator[i].pos_curr += calf_interpolator[i].pos_step;
        }

        controller->low_level_control()->publish();

        time_remaining_ -= dt_;
        if (time_remaining_ <= 0) {
            step_iterator_++;
            update_interpolators(controller);
        }
    }

    void LegBalancer::update_interpolators(Controller *controller) {
        if (step_iterator_ == config_->steps.end()) {
            printf("Reached end of steps.\n");
            controller->set_done();
            return;
        }
        Config::MotionStep &step = *step_iterator_;

        time_remaining_ = step.time + step.keep;
        float pos_step = dt_ / step.time;
        printf("Switching to the next step (time: %.4f, keep: %.4f, step: %.4f).\n",
            step.time,
            step.keep,
            pos_step
        );

        // initialize cascaded controllers
        for (int i = 0; i < 4; ++i) {
            auto &leg = controller->low_level_control()->leg(i);

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

} // namespace controllers::standheight::states
