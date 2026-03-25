#ifndef RTSC_UNITREE_ROS2_UNITREE_API
#define RTSC_UNITREE_ROS2_UNITREE_API

#include <future>
#include <mutex>
#include <map>
#include <rclcpp/node.hpp>
#include <unitree_api/msg/request.hpp>
#include <unitree_api/msg/response.hpp>

namespace interface::highlevel
{
    class UnitreeApi
    {
    protected:
        UnitreeApi(const std::string &topic, const rclcpp::Node::SharedPtr& node);
        std::future<unitree_api::msg::Response::SharedPtr> call_api(int64_t api_id, const std::string& body = std::string(""));

    private:
        void handle_response(const unitree_api::msg::Response::SharedPtr res);

        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr publisher_;
        rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr subscription_;

        std::mutex mtx_;
        int64_t request_id_ = 9000;
        std::map<uint32_t, std::promise<unitree_api::msg::Response::SharedPtr>> promises_;
    };
} // interface::highlevel

#endif // RTSC_UNITREE_ROS2_UNITREE_API