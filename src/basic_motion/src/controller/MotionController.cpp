#include "MotionController.h"
#include "states/GaitParams.h"
#include "states/InitializationState.h"
#include "states/StandParams.h"
#include "states/StateBase.h"
#include <basic_motion/msg/status.hpp>
#include <basic_motion/service.h>
#include <functional>

using StatusMsg = basic_motion::msg::Status;

namespace basic_motion::controller {
    void MotionController::timer_tick() {
        if (state_) {
            state_->timer_tick(timer_period_);
        }
    }

    void MotionController::damp_service(const srv::Damp::Request::SharedPtr req, srv::Damp::Response::SharedPtr res) {
        (void)req; // the request does not contain any relevant information
        res->status.code = state_->transition_damp() ? StatusMsg::STATUS_SUCCESS
                                                     : StatusMsg::STATUS_INVALID_TRANSITION;
    }

    void MotionController::stand_service(const srv::Stand::Request::SharedPtr req, srv::Stand::Response::SharedPtr res) {
        states::StandParams params;
        params.body_height = req->height;
        params.transition_time = req->transition_time;
        res->status.code = state_->transition_stand(params) ? StatusMsg::STATUS_SUCCESS
                                                            : StatusMsg::STATUS_INVALID_TRANSITION;
    }

    void MotionController::gait_service(const srv::Gait::Request::SharedPtr req, srv::Gait::Response::SharedPtr res) {
        states::GaitParams params;
        // TODO: Parse request
        res->status.code = state_->transition_gait(params) ? StatusMsg::STATUS_SUCCESS
                                                           : StatusMsg::STATUS_INVALID_TRANSITION;
    }

    MotionController::MotionController(const rclcpp::Node::SharedPtr &node, float timer_period)
        : node_(node), timer_period_(timer_period) {
        // start with initialization state
        state_ = std::make_shared<states::InitializationState>(this);

        // request it to change to damp state once ready
        state_->transition_damp();

        // create services
        using namespace std::placeholders;
        damp_service_ = node_->create_service<basic_motion::srv::Damp>(
            SERVICE_NAME_DAMP,
            std::bind(&MotionController::damp_service, this, _1, _2));
        stand_service_ = node_->create_service<basic_motion::srv::Stand>(
            SERVICE_NAME_STAND,
            std::bind(&MotionController::stand_service, this, _1, _2));
        gait_service_ = node_->create_service<basic_motion::srv::Gait>(
            SERVICE_NAME_GAIT,
            std::bind(&MotionController::gait_service, this, _1, _2));
    }

    rclcpp::Logger MotionController::get_logger() {
        return node_->get_logger();
    }

    std::future<void> MotionController::termination_future() {
        return termination_promise_.get_future();
    }

    void MotionController::change_state(const states::StateBase::SharedPtr &next) {
        if (state_) {
            state_->leave();
        }
        if (next) {
            state_ = next;
            next->enter();
        } else {
            termination_promise_.set_value();
        }
    }

    go2_utils::interface::LowLevelControl::SharedPtr &MotionController::low_level_control() {
        return llc_;
    }
} // namespace basic_motion::controller