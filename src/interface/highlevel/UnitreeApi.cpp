#include "UnitreeApi.h"

namespace interface::highlevel
{
    static const unitree_api::msg::Response::SharedPtr& id(const unitree_api::msg::Response::SharedPtr &msg) {
        return msg;
    };

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
        // TODO: Fixme!
        // return call_api_and_transform<const unitree_api::msg::Response::SharedPtr>(api_id, func, body);
    }

    template <typename T>
    std::future<T> UnitreeApi::call_api_and_transform(int64_t api_id, const std::function<T(const unitree_api::msg::Response::SharedPtr&)> func, const std::string &body)
    {
        // create promise for request
        std::unique_lock lock(mtx_);
        const int64_t req_id = ++request_id_;
        auto promise = TransformingPromise<T>(func);
        promises_.emplace(req_id, promise);
        auto future = promise.get_future();
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

        promises_[req_id].set_value(res);
        promises_.erase(request_id_);
    }
}
