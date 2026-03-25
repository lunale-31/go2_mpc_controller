#ifndef RTSC_UNITREE_ROS2_UNITREE_API
#define RTSC_UNITREE_ROS2_UNITREE_API

#include <future>
#include <mutex>
#include <map>
#include <rclcpp/node.hpp>
#include <unitree_api/msg/request.hpp>
#include <unitree_api/msg/response.hpp>
#include <cstdint>

namespace interface::highlevel
{
    class UnitreeApi
    {
    protected:
        UnitreeApi(const std::string &topic, const rclcpp::Node::SharedPtr &node);
        std::future<const unitree_api::msg::Response::SharedPtr> call_api(int64_t api_id, const std::string &body = std::string(""));

        template <typename T>
        std::future<T> call_api_and_transform(int64_t api_id, const std::function<T(const unitree_api::msg::Response::SharedPtr&)> func, const std::string &body = std::string(""));

    private:
        template <typename T>
        class TransformingPromise : public std::promise<const unitree_api::msg::Response::SharedPtr>
        {
        public:
            TransformingPromise(const std::function<T(const unitree_api::msg::Response::SharedPtr&)> &func) : func_(func)
            {
                // empty
            }

            std::future<T> get_future() {
                return promise_.get_future();
            }

            void set_value(const unitree_api::msg::Response::SharedPtr &resp) {
                promise_.set_value(func_(resp));
            }
        private:
            std::promise<T> promise_;
            const std::function<T(unitree_api::msg::Response::SharedPtr)> func_;
        };

        void handle_response(const unitree_api::msg::Response::SharedPtr res);

        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr publisher_;
        rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr subscription_;

        std::mutex mtx_;
        int64_t request_id_ = 9000;
        std::map<uint32_t, std::promise<const unitree_api::msg::Response::SharedPtr>> promises_;
    };
} // interface::highlevel

#endif // RTSC_UNITREE_ROS2_UNITREE_API