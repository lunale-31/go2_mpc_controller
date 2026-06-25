#include "UnitreeApi.h"

namespace interface::highlevel
{
    UnitreeApi::UnitreeApi(const std::string &topic, const rclcpp::Node::SharedPtr &node) : node_(node)
    {
        publisher_ = node->create_publisher<unitree_api::msg::Request>("/api/" + topic + "/request", 10);
        subscription_ = node->create_subscription<unitree_api::msg::Response>(
            "/api/" + topic + "/response", 10,
            std::bind(&UnitreeApi::handle_response, this, std::placeholders::_1));
    }

    std::future<const unitree_api::msg::Response::SharedPtr> UnitreeApi::call_api(int64_t api_id, const std::string &body)
    {
        const std::function<const unitree_api::msg::Response::SharedPtr(const unitree_api::msg::Response::SharedPtr&)> func = [] (const unitree_api::msg::Response::SharedPtr& msg) {return msg;};
        return call_api_and_transform<const unitree_api::msg::Response::SharedPtr>(api_id, func, body);
    }

    void UnitreeApi::handle_response(const unitree_api::msg::Response::SharedPtr res)
    {
        std::lock_guard guard(mtx_);
        const int64_t req_id = res->header.identity.id;

        // check that promise exists
        if (!promises_.contains(req_id))
        {
            RCLCPP_WARN(node_->get_logger(), "No promise found for request %ld", req_id);
            return;
        }

        promises_[req_id]->set_value(res);
        promises_.erase(request_id_);
    }
}
