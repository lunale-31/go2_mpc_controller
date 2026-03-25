//
// Created by ubuntu on 3/23/26.
//

#include "HighLevelControl.h"
#include <string>
#include <chrono>

static const std::string API_TOPIC = "sport";
static const std::string RESPONSE_TOPIC = "/api/sport/response";

static constexpr int64_t API_ID_DAMP = 1001;
static constexpr int64_t API_ID_STOP_MOVE = 1003;
static constexpr int64_t API_ID_STAND_UP = 1004;
static constexpr int64_t API_ID_SIT_DOWN = 1005;

using namespace std::chrono_literals;
static constexpr auto RESPONSE_TIMEOUT = 500ms;

namespace interface
{
    HighLevelControl::HighLevelControl(const rclcpp::Node::SharedPtr &node) : UnitreeApi(API_TOPIC, node)
    {
        // empty
    }

    std::future<unitree_api::msg::Response::SharedPtr> HighLevelControl::damp()
    {
        return call_api(API_ID_DAMP);
    }

    std::future<unitree_api::msg::Response::SharedPtr> HighLevelControl::stop_move()
    {
        return call_api(API_ID_STOP_MOVE);
    }

    std::future<unitree_api::msg::Response::SharedPtr> HighLevelControl::stand_up()
    {
        return call_api(API_ID_STAND_UP);
    }

    std::future<unitree_api::msg::Response::SharedPtr> HighLevelControl::sit_down()
    {
        return call_api(API_ID_SIT_DOWN);
    }
} // interface
