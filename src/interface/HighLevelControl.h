//
// Created by ubuntu on 3/23/26.
//

#ifndef RTSC_UNITREE_ROS2_HIGHCOMMAND_H
#define RTSC_UNITREE_ROS2_HIGHCOMMAND_H

#include <rclcpp/node.hpp>
#include <unitree_api/msg/request.hpp>
#include <unitree_api/msg/response.hpp>
#include <mutex>
#include <map>
#include <future>

namespace interface
{
    class HighLevelControl
    {
    public:
        explicit HighLevelControl(const rclcpp::Node::SharedPtr& node);

        std::future<bool> stand_up();
        std::future<bool> sit_down();

    private:
        std::future<bool> call_api(int64_t api_id, const std::string& body = std::string(""));
        void handle_response(const unitree_api::msg::Response& res);

        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr publisher_;
        rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr subscription_;

        std::mutex mtx_;
        int64_t request_id_ = 9000;
        std::map<uint32_t, std::promise<bool>> promises_;
    };
} // interface

#endif //RTSC_UNITREE_ROS2_HIGHCOMMAND_H
