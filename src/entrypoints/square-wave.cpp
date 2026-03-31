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
    double tau_min_, tau_max_, dq_;
    unsigned switch_period_;

    // state
    common::PidController::SharedPtr pid_;
    unsigned cycle_;
    bool direction_ = true;
    unsigned periods_;

    // startup time
    unsigned startup_ = 400;

    FILE *logfile_;

    std::promise<void> done_;

public:
    Monitor(rclcpp::Node::SharedPtr &node) : node_(node) {
        // initialize low-level control and timer
        llc_ = std::make_shared<interface::LowLevelControl>(node);
        timer_ = node->create_wall_timer(control_period, std::bind(&Monitor::timer_tick, this));
        
        // bring calf to start position
        auto calf = llc_->backLeft()->calf();
        calf->mode(1);
        calf->kp(60.0);
        calf->kd(5.0);
        calf->q(-2.6);
        
        // fixate hip
        auto hip = llc_->backLeft()->hip();
        hip->mode(1);
        hip->kp(60.0);
        hip->kd(5.0);
        hip->q(0.0);
        
        // fixate thigh
        auto thigh = llc_->backLeft()->thigh();
        thigh->mode(1);
        thigh->kp(60.0);
        thigh->kd(5.0);
        thigh->q(3.4);
        
        joint_ = calf;
        
        // read config
        auto config = YAML::LoadFile("../params.yaml")["square_wave"];
        tau_min_ = config["tau_min"].as<float>();
        tau_max_ = config["tau_max"].as<float>();
        dq_ = config["dq"].as<float>();
        switch_period_ = config["switch_period"].as<unsigned>();
        periods_ = config["periods"].as<unsigned>();

        // initialize pid controller
        pid_ = std::make_shared<common::PidController>(
            config["kp"].as<float>(),
            config["ki"].as<float>(),
            config["kd"].as<float>());
        pid_->setpoint(dq_);

        // initialize plot file
        logfile_ = fopen(config["logfile"].as<std::string>().c_str(), "w");
        fprintf(logfile_, "dq_r,dq,error,q,tau_est,signal\n");
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

        auto torque_signal = pid_->control(state.dq, 0.002, tau_min_, tau_max_);

        // write state to plot file
        fprintf(
            logfile_,
            "%f, %f, %f, %f, %f, %f\n",
            pid_->setpoint(),
            state.dq,
            state.dq - pid_->setpoint(),
            state.q,
            state.tau_est,
            torque_signal);

        joint_->mode(1);
        joint_->kp(0.0);
        joint_->kd(0.0);
        joint_->tau(torque_signal);
        llc_->publish();

        // step control
        cycle_++;
        if (cycle_ >= switch_period_) {
            cycle_ = 0;
            direction_ = !direction_;
            pid_->setpoint(direction_ ? dq_ : -dq_);
            periods_--;
            if (!periods_) {
                done_.set_value();
            } else {
                std::cout << "Completed period (" << periods_ << " remaining)." << std::endl;
            }
        }
    };

    std::future<void> done_future() {
        return done_.get_future();
    }

    using UniquePtr = std::unique_ptr<Monitor>;
    using SharedPtr = std::shared_ptr<Monitor>;
};

/**
 * Main entry point for sine wave experiment
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