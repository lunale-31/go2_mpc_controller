#include <chrono>
#include <common/go_constants.h>
#include <cstdio>
#include <future>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <yaml-cpp/yaml.h>

#include "../common/PidController.h"
#include "../interface/LowLevelControl.h"
#include "../interface/lowlevel/Joint.h"

using namespace std::chrono_literals;

constexpr auto control_period = 2ms;

class Monitor {
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr timer_;
    interface::LowLevelControl::SharedPtr llc_;
    interface::lowlevel::Joint::SharedPtr joint_;

    // config parameters
    double tau_min_, tau_max_;

    // state
    common::PidController::SharedPtr pos_pid_;
    common::PidController::SharedPtr velo_pid_;
    float pos_min_, pos_max_;
    float current_ = 0.0f, step_, stop_;
    unsigned outer_factor_, outer_current_;

    // startup time
    unsigned startup_ = 400;

    FILE *logfile_;

    std::promise<void> done_;

public:
    Monitor(rclcpp::Node::SharedPtr &node) : node_(node) {
        // initialize low-level control and timer
        llc_ = std::make_shared<interface::LowLevelControl>(node);
        timer_ = node->create_wall_timer(control_period, std::bind(&Monitor::timer_tick, this));

        // read config
        auto config = YAML::LoadFile("../params.yaml")["cascaded_sine_wave"];
        tau_min_ = config["tau_min"].as<float>();
        tau_max_ = config["tau_max"].as<float>();
        pos_min_ = config["pos_min"].as<float>();
        pos_max_ = config["pos_max"].as<float>();
        outer_factor_ = config["outer_factor"].as<unsigned>();
        unsigned period_samples = config["period_samples"].as<unsigned>();
        unsigned periods = config["periods"].as<unsigned>();

        step_ = (2.0f * M_PI) / static_cast<float>(period_samples);
        stop_ = 2.0f * M_PI * static_cast<float>(periods);

        // initialize pid controllers
        pos_pid_ = std::make_shared<common::PidController>(
            config["position"]["kp"].as<float>(),
            config["position"]["ki"].as<float>(),
            config["position"]["kd"].as<float>());

        velo_pid_ = std::make_shared<common::PidController>(
            config["velocity"]["kp"].as<float>(),
            config["velocity"]["ki"].as<float>(),
            config["velocity"]["kd"].as<float>());

        // initialize plot file
        logfile_ = fopen(config["logfile"].as<std::string>().c_str(), "w");
        fprintf(logfile_, "q_r,q,q_error,dq_r,dq,dq_error,signal\n");

        // bring calf to start position
        auto calf = llc_->backRight()->calf();
        calf->mode(1);
        calf->kp(60.0);
        calf->kd(5.0);
        calf->q(-1.85);

        // fixate hip
        auto hip = llc_->backRight()->hip();
        hip->mode(1);
        hip->kp(60.0);
        hip->kd(5.0);
        hip->q(0.0);

        // fixate thigh
        auto thigh = llc_->backRight()->thigh();
        thigh->mode(0);
        thigh->kp(60.0);
        thigh->kd(5.0);
        thigh->q(1.6);

        joint_ = thigh;
    }

    ~Monitor() {
        fclose(logfile_);
    }

    void timer_tick() {
        // startup
        if (startup_) {
            startup_--;
            if (!startup_) {
                std::cout << "Startup completed." << std::endl;
            } else {
                llc_->publish();
            }
            return;
        }

        const auto &state = joint_->state();
        pos_pid_->setpoint((sin(current_) + 1.0f) / 2.0f * (pos_max_ - pos_min_) + pos_min_);
        if (outer_current_ == 0) {
            const float velocity_setpoint = pos_pid_->control(state.q, 0.002f * outer_factor_);
            velo_pid_->setpoint(velocity_setpoint);
        }
        if (++outer_current_ >= outer_factor_) {
            outer_current_ = 0;
        }

        const float torque_signal = velo_pid_->control(state.dq, 0.002f, tau_min_, tau_max_);

        // write state to plot file
        fprintf(
            logfile_,
            "%f, %f, %f, %f, %f, %f, %f\n",
            pos_pid_->setpoint(),
            state.q,
            state.q - velo_pid_->setpoint(),
            velo_pid_->setpoint(),
            state.dq,
            state.dq - pos_pid_->setpoint(),
            torque_signal);

        joint_->mode(1);
        joint_->kp(0.0);
        joint_->kd(0.0);
        joint_->tau(torque_signal);
        llc_->publish();

        // step control
        current_ += step_;
        if (current_ >= stop_) {
            done_.set_value();
        }
    };

    std::future<void> done_future() {
        return done_.get_future();
    }

    using UniquePtr = std::unique_ptr<Monitor>;
    using SharedPtr = std::shared_ptr<Monitor>;
};

/**
 * Main entry point for cascaded sine wave experiment
 * @param argc Number of program arguments
 * @param argv Program arguments
 */
int main(const int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("monitor");
    Monitor::UniquePtr monitor = std::make_unique<Monitor>(node);

    // create node and subscribe to /lowcmd
    std::this_thread::sleep_for(200ms);

    // execute node
    auto exec = rclcpp::executors::SingleThreadedExecutor();
    exec.add_node(node);
    exec.spin_until_future_complete(monitor->done_future());

    // stop and return result
    rclcpp::shutdown();
    return 0;
}