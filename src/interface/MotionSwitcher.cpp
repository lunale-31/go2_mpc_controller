#include "MotionSwitcher.h"
#include <nlohmann/json.hpp>
#include <string>
using json = nlohmann::json;

// define API topic (/api/API_TOPIC/request, /api/API_TOPIC/response)
static const std::string API_TOPIC = "motion_switcher";

// define Unitree API ids
static constexpr int64_t API_SET_SILENT = 1004;
static constexpr int64_t API_GET_SILENT = 1005;

static bool check_status_code(const unitree_api::msg::Response::SharedPtr &msg) {
    return msg->header.status.code == 0;
}

namespace interface {
    MotionSwitcher::MotionSwitcher(const rclcpp::Node::SharedPtr &node) : UnitreeApi(API_TOPIC, node) {
        // empty
    }

    std::future<bool> MotionSwitcher::get_silent() {
        auto transformer = [](const unitree_api::msg::Response::SharedPtr &msg) {
            json resp = json::parse(msg->data);
            return resp["silent"].get<bool>();
        };
        return call_api_and_transform<bool>(API_GET_SILENT, transformer);
    }

    std::future<bool> MotionSwitcher::set_silent(bool silent) {
        json req = {{"silent", silent}};
        return call_api_and_transform<bool>(API_SET_SILENT, check_status_code, req.dump());
    }
} // namespace interface
