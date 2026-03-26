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
    /**
     * Communication wrapper for interfacing with Unitree's high-level request-response API 
     */
    class UnitreeApi
    {
    protected:
        /**
         * Constructor.
         * @param topic The API name to interact with (publishes and subscribes to `/api/TOPIC/request` and `/api/TOPIC/response`)
         * @param node The RCLCPP node to create the publisher and subscription on
         */
        UnitreeApi(const std::string &topic, const rclcpp::Node::SharedPtr &node);

        /**
         * Perform an API call and return the response message
         * @param api_id The API endpoint identifier
         * @param body (optional) A message payload, often in JSON format
         * @returns A future containing the response message
         */
        std::future<const unitree_api::msg::Response::SharedPtr> call_api(int64_t api_id, const std::string &body = std::string(""));

        /**
         * Performs an API call and returns a transformed version of the response message
         * @param T The desired return type
         * @param api_id The API endpoint identifier
         * @param func A function transforming the response message into the desired target type
         * @param body (optional) A message payload, often in JSON format
         * @returns A future containing the transformed response message
         */
        template <typename T>
        inline std::future<T> call_api_and_transform(int64_t api_id, const std::function<T(const unitree_api::msg::Response::SharedPtr &)> func, const std::string &body = std::string(""))
        {
            // create promise for request
            std::unique_lock lock(mtx_);
            const int64_t req_id = ++request_id_;
            auto promise = std::make_shared<TransformingPromise<T>>(func);
            auto future = promise->get_future();
            promises_[req_id] = promise;
            lock.unlock();

            // build and perform request
            unitree_api::msg::Request req;
            req.header.identity.api_id = api_id;
            req.header.identity.id = req_id;
            req.parameter = body;
            RCLCPP_INFO(node_->get_logger(), "Sending request %ld to api %ld (body: '%s')", req_id, api_id, body.c_str());
            publisher_->publish(req);
            return future;
        }

    private:
        class TransformingPromiseBase
        {
        public:
            virtual void set_value(const unitree_api::msg::Response::SharedPtr &resp) = 0;
        };

        template <typename T>
        class TransformingPromise : public TransformingPromiseBase
        {
        public:
            TransformingPromise(const std::function<T(const unitree_api::msg::Response::SharedPtr &)> &func) : func_(func)
            {
                // empty
            }

            std::future<T> get_future()
            {
                return promise_.get_future();
            }

            void set_value(const unitree_api::msg::Response::SharedPtr &resp) override
            {
                try {
                    promise_.set_value(func_(resp));
                } catch (std::exception &_) {
                    promise_.set_exception(std::current_exception());
                }
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
        std::map<uint32_t, std::shared_ptr<TransformingPromiseBase>> promises_;
    };
} // interface::highlevel

#endif // RTSC_UNITREE_ROS2_UNITREE_API