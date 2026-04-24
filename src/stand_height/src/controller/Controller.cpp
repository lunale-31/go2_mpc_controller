#include "Controller.h"
#include <chrono>
#include <functional>
#include <go2_utils/kinematics.h>
#include <go2_utils/robot.h>
#include <stand_height/service.h>

static constexpr unsigned CONTROL_PERIOD_MS = 2;

static constexpr float THIGH_MIN = 0.0f, THIGH_MAX = 2.5f;

static constexpr float SIT_DOWN_THRESHOLD = 0.18f;

static constexpr JointPose sit_down_pose_front{
    .hip_q = -0.05f,
    .thigh_q = 1.3f,
    .calf_q = -2.8f};

static constexpr JointPose sit_down_pose_back{
    .hip_q = 0.3f,
    .thigh_q = 1.31f,
    .calf_q = -2.81f};

Controller::Controller(const rclcpp::Node::SharedPtr &node) : node_(node) {
    using namespace std::placeholders;
    using namespace std::chrono_literals;

    service_ = node_->create_service<stand_height::srv::StandHeight>(
        stand_height::SERVICE_NAME,
        std::bind(&Controller::service_request, this, _1, _2));

    timer_ = node_->create_wall_timer(std::chrono::milliseconds(CONTROL_PERIOD_MS), std::bind(&Controller::timer_tick, this));

    llc_ = std::make_shared<go2_utils::interface::LowLevelControl>(node);
}

void Controller::timer_tick() {
    if (is_initialized_) {
        control_tick();
    } else {
        initializion_tick();
    }
}

void Controller::initializion_tick() {
    if (llc_->was_state_received()) {
        for (unsigned i = 0; i < go2_utils::robot::LEG_COUNT; ++i) {
            auto &leg = llc_->leg(i);

            // measure starting pose
            JointPose::fromLegState(poses[i], leg);

            // prepare motor commands
            auto &hip_cmd = leg->hip()->cmd();
            hip_cmd.mode = 1;
            hip_cmd.kp = 60;
            hip_cmd.kd = 5;

            auto &thigh_cmd = leg->thigh()->cmd();
            thigh_cmd.mode = 1;
            thigh_cmd.kp = 60;
            thigh_cmd.kd = 5;

            auto &calf_cmd = leg->calf()->cmd();
            calf_cmd.mode = 1;
            calf_cmd.kp = 60;
            calf_cmd.kd = 5;
        }

        RCLCPP_INFO(node_->get_logger(), "Server is ready.");
        is_initialized_ = true;
        return;
    }
}

void Controller::control_tick() {
    for (unsigned i = 0; i < go2_utils::robot::LEG_COUNT; ++i) {
        if (pose_interpolators[i]) {
            pose_interpolators[i]->update(0.001f * CONTROL_PERIOD_MS);
            if (pose_interpolators[i]->finished()) {
                pose_interpolators[i] = nullptr;
            }
        }
        JointPose::toLegCommand(poses[i], llc_->leg(i));
    }

    llc_->publish();
}

void Controller::service_request(
    const stand_height::srv::StandHeight::Request::SharedPtr request,
    stand_height::srv::StandHeight::Response::SharedPtr response) {

    if (!is_initialized_) {
        RCLCPP_WARN(node_->get_logger(),
                    "Server is not yet ready.");
        response->status = stand_height::srv::StandHeight::Response::STATUS_SERVER_NOT_READY;
        return;
    }

    if (request->height < 0.0f || request->transition_time < 0.0f) {
        response->status = stand_height::srv::StandHeight::Response::STATUS_INVALID_REQUEST;
        return;
    }

    RCLCPP_INFO(node_->get_logger(),
                "Received request for height %.4f and x offset %.4f over transition time %.4f.",
                request->height, request->x_offset, request->transition_time);

    if (request->height < SIT_DOWN_THRESHOLD) {
        const auto joint_configurations = go2_utils::kinematics::inverse(
            Eigen::Vector3f(0.0f, go2_utils::robot::L_1, -SIT_DOWN_THRESHOLD),
            go2_utils::robot::LegSide::LEFT);

        for (const auto &conf : joint_configurations) {
            if (conf.y() >= THIGH_MIN && conf.y() <= THIGH_MAX) {
                JointPose jp_to;
                jp_to.hip_q = conf.x();
                jp_to.thigh_q = conf.y();
                jp_to.calf_q = conf.z();

                const float ratio = request->height / SIT_DOWN_THRESHOLD;
                JointPose jp_fl = JointPose::interpolate(jp_to, sit_down_pose_front, ratio),
                          jp_fr = JointPose::interpolate(jp_to, sit_down_pose_front, ratio),
                          jp_bl = JointPose::interpolate(jp_to, sit_down_pose_back, ratio),
                          jp_br = JointPose::interpolate(jp_to, sit_down_pose_back, ratio);

                jp_fr.hip_q *= -1.0f;
                jp_br.hip_q *= -1.0f;

                pose_interpolators[go2_utils::robot::LEG_FRONT_LEFT] = std::make_unique<JointPoseInterpolation>(
                    poses[go2_utils::robot::LEG_FRONT_LEFT], jp_fl,
                    poses[go2_utils::robot::LEG_FRONT_LEFT], request->transition_time);
                pose_interpolators[go2_utils::robot::LEG_FRONT_RIGHT] = std::make_unique<JointPoseInterpolation>(
                    poses[go2_utils::robot::LEG_FRONT_RIGHT], jp_fr,
                    poses[go2_utils::robot::LEG_FRONT_RIGHT], request->transition_time);
                pose_interpolators[go2_utils::robot::LEG_BACK_LEFT] = std::make_unique<JointPoseInterpolation>(
                    poses[go2_utils::robot::LEG_BACK_LEFT], jp_bl,
                    poses[go2_utils::robot::LEG_BACK_LEFT], request->transition_time);
                pose_interpolators[go2_utils::robot::LEG_BACK_RIGHT] = std::make_unique<JointPoseInterpolation>(
                    poses[go2_utils::robot::LEG_BACK_RIGHT], jp_br,
                    poses[go2_utils::robot::LEG_BACK_RIGHT], request->transition_time);

                response->status = stand_height::srv::StandHeight::Response::STATUS_SUCCESS;
                return;
            }
        }
        RCLCPP_WARN(node_->get_logger(),
                    "Could not find valid joint configuration for requested sitting height.");
        response->status = stand_height::srv::StandHeight::Response::STATUS_OUT_OF_REACH;
    }

    const auto joint_configurations = go2_utils::kinematics::inverse(
        Eigen::Vector3f(request->x_offset, go2_utils::robot::L_1, -(request->height)),
        go2_utils::robot::LegSide::LEFT);

    for (const auto &conf : joint_configurations) {
        if (conf.y() >= THIGH_MIN && conf.y() <= THIGH_MAX) {
            JointPose jp_to;
            jp_to.hip_q = conf.x();
            jp_to.thigh_q = conf.y();
            jp_to.calf_q = conf.z();
            for (unsigned i = 0; i < go2_utils::robot::LEG_COUNT; ++i) {
                pose_interpolators[i] = std::make_unique<JointPoseInterpolation>(
                    poses[i], jp_to, poses[i], request->transition_time);
            }
            response->status = stand_height::srv::StandHeight::Response::STATUS_SUCCESS;
            return;
        }
    }

    RCLCPP_WARN(node_->get_logger(),
                "Could not find valid joint configuration for requested standing height.");
    response->status = stand_height::srv::StandHeight::Response::STATUS_OUT_OF_REACH;
}
