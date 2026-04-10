#include <chrono>
#include <common/go_constants.h>
#include <cstdio>
#include <future>
#include <map>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <yaml-cpp/yaml.h>

#include "../common/PidController.h"
#include "../interface/LowLevelControl.h"
#include "../interface/lowlevel/Joint.h"

using namespace std::chrono_literals;

constexpr auto control_period = 2ms;

/**
 * Experiment Config
 */
struct Config {
    enum Leg {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    };

    enum Joint {
        HIP,
        THIGH,
        CALF
    };

    std::string logfile_path;

    float pos_k, pos_ti, pos_td, pos_n, pos_beta, pos_tr;
    float velo_k, velo_ti, velo_td, velo_n, velo_beta, velo_tr;

    float pos_min, pos_max;
    float tau_min, tau_max;
    unsigned periods, ticks_per_period, ms_per_tick, outer_factor;
    Leg leg;
    Joint joint;
    float hip_initial, thigh_initial, calf_initial;
};

class Monitor {
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr timer_;
    interface::LowLevelControl::SharedPtr llc_;
    interface::lowlevel::Joint::SharedPtr joint_;

    std::shared_ptr<Config> config_;

    // state
    common::PidController::SharedPtr pos_pid_;
    common::PidController::SharedPtr velo_pid_;
    float current_ = 0.0f, step_, stop_;
    unsigned outer_current_;

    // startup time
    unsigned startup_ = 400;

    FILE *logfile_;

    std::promise<void> done_;

public:
    Monitor(rclcpp::Node::SharedPtr &node, std::shared_ptr<Config> &config) : node_(node), config_(config) {
        // initialize low-level control and timer
        llc_ = std::make_shared<interface::LowLevelControl>(node);
        timer_ = node->create_wall_timer(
            std::chrono::milliseconds(config->ms_per_tick),
            std::bind(&Monitor::timer_tick, this));

        // pick leg
        interface::lowlevel::Leg::SharedPtr leg;
        switch (config->leg) {
            case Config::Leg::FRONT_LEFT:
                leg = llc_->frontLeft();
                break;
            case Config::Leg::FRONT_RIGHT:
                leg = llc_->frontRight();
                break;
            case Config::Leg::BACK_LEFT:
                leg = llc_->backLeft();
                break;
            case Config::Leg::BACK_RIGHT:
                leg = llc_->backRight();
                break;
            default:
                throw new std::invalid_argument("Invalid leg value provided in config structure.");
        }

        // pick joint
        switch (config->joint) {
            case Config::Joint::HIP:
                joint_ = leg->hip();
                break;
            case Config::Joint::THIGH:
                joint_ = leg->thigh();
                break;
            case Config::Joint::CALF:
                joint_ = leg->calf();
                break;
            default:
                break;
        }

        // initial position of hip
        auto hip = leg->hip();
        hip->mode(std::isnan(config->hip_initial) ? 0 : 1);
        hip->kp(60.0);
        hip->kd(5.0);
        hip->q(std::isnan(config->hip_initial) ? 0.0f : config->hip_initial);

        // initial position of thigh
        auto thigh = leg->thigh();
        thigh->mode(std::isnan(config->thigh_initial) ? 0 : 1);
        thigh->kp(60.0);
        thigh->kd(5.0);
        thigh->q(std::isnan(config->thigh_initial) ? 0.0f : config->thigh_initial);

        // initial position of calf
        auto calf = leg->calf();
        calf->mode(std::isnan(config->calf_initial) ? 0 : 1);
        calf->kp(60.0);
        calf->kd(5.0);
        calf->q(std::isnan(config->calf_initial) ? 0.0f : config->calf_initial);

        // initialize state
        step_ = (2.0f * M_PI) / static_cast<float>(config->ticks_per_period);
        stop_ = 2.0f * M_PI * static_cast<float>(config->periods);

        // initialize pid controllers
        pos_pid_ = std::make_shared<common::PidController>(
            config_->pos_k, config_->pos_ti, config_->pos_td,
            config_->pos_n, config_->pos_beta, config_->pos_tr
        );
        velo_pid_ = std::make_shared<common::PidController>(
            config_->velo_k, config_->velo_ti, config_->velo_td,
            config_->velo_n, config_->velo_beta, config_->velo_tr
        );

        // initialize plot file
        logfile_ = fopen(config->logfile_path.c_str(), "w");
        fprintf(logfile_, "q_r,q,q_error,dq_r,dq,dq_error,signal\n");
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
        pos_pid_->setpoint((sin(current_) + 1.0f) / 2.0f * (config_->pos_max - config_->pos_min) + config_->pos_min);
        if (outer_current_ == 0) {
            const float velocity_setpoint = pos_pid_->control(state.q, 0.001f * config_->ms_per_tick * config_->outer_factor);
            velo_pid_->setpoint(velocity_setpoint);
        }
        if (++outer_current_ >= config_->outer_factor) {
            outer_current_ = 0;
        }

        const float torque_signal = velo_pid_->control(state.dq, 0.001f * config_->ms_per_tick, config_->tau_min, config_->tau_max);

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
 * Load and parse config
 */
std::shared_ptr<Config> load_config(const char *config_path) {
    auto config = std::make_shared<Config>();
    auto config_file = YAML::LoadFile(config_path)["cascaded_sine_wave"];
    config->logfile_path = config_file["logfile"].as<std::string>();
    auto position = config_file["position"];
    config->pos_k = position["K"].as<float>();
    config->pos_ti = position["Ti"].as<float>();
    config->pos_td = position["Td"].as<float>();
    config->pos_n = position["N"].as<float>();
    config->pos_beta = position["Beta"].as<float>();
    config->pos_tr = position["Tr"].as<float>();
    auto velocity = config_file["velocity"];
    config->velo_k = velocity["K"].as<float>();
    config->velo_ti = velocity["Ti"].as<float>();
    config->velo_td = velocity["Td"].as<float>();
    config->velo_n = velocity["N"].as<float>();
    config->velo_beta = velocity["Beta"].as<float>();
    config->velo_tr = velocity["Tr"].as<float>();
    config->pos_min = config_file["pos_min"].as<float>();
    config->pos_max = config_file["pos_max"].as<float>();
    config->tau_min = config_file["tau_min"].as<float>();
    config->tau_max = config_file["tau_max"].as<float>();
    config->periods = config_file["periods"].as<unsigned>();
    config->ticks_per_period = config_file["ticks_per_period"].as<unsigned>();
    config->ms_per_tick = config_file["ms_per_tick"].as<unsigned>();
    config->outer_factor = config_file["outer_factor"].as<unsigned>();

    // Parse optional initial joint positions
    auto initial = config_file["initial"];
    auto hip_initial = initial["hip"];
    config->hip_initial = (!hip_initial.IsDefined() || hip_initial.IsNull()) ? NAN : hip_initial.as<float>();
    auto thigh_initial = initial["thigh"];
    config->thigh_initial = (!thigh_initial.IsDefined() || thigh_initial.IsNull()) ? NAN : thigh_initial.as<float>();
    auto calf_initial = initial["calf"];
    config->calf_initial = (!calf_initial.IsDefined() || calf_initial.IsNull()) ? NAN : calf_initial.as<float>();

    // Parse leg to be controlled
    const std::map<std::string, Config::Leg> legs = {
        {"front_left", Config::Leg::FRONT_LEFT},
        {"front_right", Config::Leg::FRONT_RIGHT},
        {"back_left", Config::Leg::BACK_LEFT},
        {"back_right", Config::Leg::BACK_RIGHT}};

    if (auto leg_str = config_file["leg"].as<std::string>(); legs.contains(leg_str)) {
        config->leg = legs.find(leg_str)->second;
    } else {
        throw new std::invalid_argument("Invalid config value provided for leg.");
    }

    // Parse joint on the leg to be controlled
    const std::map<std::string, Config::Joint> joints = {
        {"hip", Config::Joint::HIP},
        {"thigh", Config::Joint::THIGH},
        {"calf", Config::Joint::CALF}};

    if (auto joint_str = config_file["joint"].as<std::string>(); joints.contains(joint_str)) {
        config->joint = joints.find(joint_str)->second;
    } else {
        throw new std::invalid_argument("Invalid config value provided for joint.");
    }

    return config;
}

/**
 * Main entry point for cascaded sine wave experiment
 * @param argc Number of program arguments
 * @param argv Program arguments
 */
int main(const int argc, char *argv[]) {
    // Load and parse config
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <config_file>" << std::endl;
        return -1;
    }
    auto config = load_config(argv[1]);

    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("monitor");
    Monitor::UniquePtr monitor = std::make_unique<Monitor>(node, config);

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