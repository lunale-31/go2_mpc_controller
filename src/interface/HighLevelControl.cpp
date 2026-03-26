#include "HighLevelControl.h"
#include <string>

// define API topic (/api/API_TOPIC/request, /api/API_TOPIC/response)
static const std::string API_TOPIC = "sport";

// define Unitree API ids
static constexpr int64_t API_ID_DAMP = 1001;
static constexpr int64_t API_ID_STOP_MOVE = 1003;
static constexpr int64_t API_ID_STAND_UP = 1004;
static constexpr int64_t API_ID_SIT_DOWN = 1005;

static bool check_status_code(const unitree_api::msg::Response::SharedPtr &msg) {
    return msg->header.status.code == 0;
}

namespace interface {
    HighLevelControl::HighLevelControl(const rclcpp::Node::SharedPtr &node) : UnitreeApi(API_TOPIC, node) {
        // empty
    }

    std::future<bool> HighLevelControl::damp() {
        return call_api_and_transform<bool>(API_ID_DAMP, check_status_code);
    }

    std::future<bool> HighLevelControl::stop_move() {
        return call_api_and_transform<bool>(API_ID_STOP_MOVE, check_status_code);
    }

    std::future<bool> HighLevelControl::stand_up() {
        return call_api_and_transform<bool>(API_ID_STAND_UP, check_status_code);
    }

    std::future<bool> HighLevelControl::sit_down() {
        return call_api_and_transform<bool>(API_ID_SIT_DOWN, check_status_code);
    }
} // namespace interface
