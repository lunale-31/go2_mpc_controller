//
// Created by ubuntu on 3/23/26.
//

#include "HighLevelControl.h"
#include <string>

// define API topic (/api/API_TOPIC/request, /api/API_TOPIC/response)
static const std::string API_TOPIC = "motion_switcher";

// define Unitree API ids
static constexpr int64_t API_SET_SILENT = 1004;
static constexpr int64_t API_GET_SILENT = 1005;

namespace interface
{
    MotionSwitcher::MotionSwitcher(const rclcpp::Node::SharedPtr &node) : UnitreeApi(API_TOPIC, node)
    {
        // empty
    }

    std::future<const unitree_api::msg::Response::SharedPtr> MotionSwitcher::get_silent()
    {
        return call_api(API_GET_SILENT);
    }

    std::future<const unitree_api::msg::Response::SharedPtr> MotionSwitcher::set_silent(bool silent)
    {
        // TODO: Build JSON object!
        return call_api(API_SET_SILENT, "{}");
    }
} // interface
