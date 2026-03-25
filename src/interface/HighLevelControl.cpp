//
// Created by ubuntu on 3/23/26.
//

#include "HighLevelControl.h"
#include <string>
#include <chrono>

static const std::string REQUEST_TOPIC = "/api/sport/request";
static const std::string RESPONSE_TOPIC = "/api/sport/response";

static constexpr int64_t API_ID_STAND_UP = 1004;
static constexpr int64_t API_ID_SIT_DOWN = 1005;

using namespace std::chrono_literals;
static constexpr auto RESPONSE_TIMEOUT = 500ms;

namespace interface
{
    HighLevelControl::HighLevelControl(const rclcpp::Node::SharedPtr &node) : node_(node)
    {
        publisher_ = node->create_publisher<unitree_api::msg::Request>(REQUEST_TOPIC, 10);
        subscription_ = node->create_subscription<unitree_api::msg::Response>(
            RESPONSE_TOPIC, 10,
            std::bind(&HighLevelControl::handle_response, this, std::placeholders::_1)
        );

        // Wait for topics to be ready
    }

    std::future<bool> HighLevelControl::stand_up()
    {
        return call_api(API_ID_STAND_UP);
    }

    std::future<bool> HighLevelControl::sit_down()
    {
        return call_api(API_ID_SIT_DOWN);
    }

    std::future<bool> HighLevelControl::call_api(const int64_t api_id, const std::string& body)
    {
        // create promise for request
        std::unique_lock lock(mtx_);
        const int64_t req_id = ++request_id_;
        promises_.emplace(req_id, std::promise<bool>());
        auto future = promises_[req_id].get_future();
        lock.unlock();

        // build and perform request
        unitree_api::msg::Request req;
        req.header.identity.api_id = api_id;
        req.header.identity.id = req_id;
        req.parameter = body;
        RCLCPP_INFO(node_->get_logger(), "Sending request %ld to api %ld", req_id, api_id);
        publisher_->publish(req);
        return future;
    }

    void HighLevelControl::handle_response(const unitree_api::msg::Response& res)
    {
        std::lock_guard guard(mtx_);
        const int64_t req_id = res.header.identity.id;

        // check that promise exists
        if (!promises_.contains(req_id))
        {
            RCLCPP_WARN(node_->get_logger(), "No promise found for request %ld", req_id);
            return;
        }

        // TODO: Remove this
        RCLCPP_INFO(
            node_->get_logger(), "Received response for request %ld (api_id: %ld, status: %d)",
            req_id, res.header.identity.api_id, res.header.status.code
        );

        promises_[req_id].set_value(res.header.status.code == 0);
        promises_.erase(request_id_);
    }
} // interface
