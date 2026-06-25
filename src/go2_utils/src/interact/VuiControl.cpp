#include "../lib/nlohmann/json.hpp"
#include <go2_utils/interact/VuiControl.h>
#include <string>

using json = nlohmann::json;

// define API topic (/api/API_TOPIC/request, /api/API_TOPIC/response)
static const std::string API_TOPIC = "vui";

// define Unitree API ids
static constexpr int64_t API_SET_COLOR = 1007;

static bool check_status_code(const unitree_api::msg::Response::SharedPtr &msg) {
    return msg->header.status.code == 0;
}


namespace go2_utils::interact {
    static std::string color_str(VuiControl::LedColor color) {
        switch (color) {
            case VuiControl::LedColor::WHITE:
                return "white";
            case VuiControl::LedColor::RED:
                return "red";
            case VuiControl::LedColor::YELLOW:
                return "yellow";
            case VuiControl::LedColor::GREEN:
                return "green";
            case VuiControl::LedColor::CYAN:
                return "cyan";
            case VuiControl::LedColor::BLUE:
                return "blue";
            case VuiControl::LedColor::PURPLE:
                return "purple";
            default:
                return "white";
        }
    }

    VuiControl::VuiControl(const rclcpp::Node::SharedPtr &node) : UnitreeApi(API_TOPIC, node) {
        // empty
    }

    std::future<bool> VuiControl::set_led_color(LedColor color, int time, int flash_cycle) {
        json req = {{"color", color_str(color)}};
        if (time >= 0) {
            req["time"] = time;
        }
        if (flash_cycle >= 0) {
            req["flash_cycle"] = flash_cycle;
        }
        return call_api_and_transform<bool>(API_SET_COLOR, check_status_code, req.dump());
    } 
    
    /*
         std::future<bool> VuiControl::get_silent() {
             auto transformer = [](const unitree_api::msg::Response::SharedPtr &msg) {
                 json resp = json::parse(msg->data);
                 return resp["silent"].get<bool>();
             };
             return call_api_and_transform<bool>(API_GET_SILENT, transformer);
         }
         */
} // namespace go2_utils::interact
