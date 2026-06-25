#include "StateBase.h"
#include "../MotionController.h"

namespace basic_motion::controller::states {
    rclcpp::Logger StateBase::get_logger() {
        return controller_->get_logger();
    }

    StateBase::StateBase(MotionController *controller)
        : controller_(controller), llc_(controller->low_level_control())
    {
        // empty
    }
} // namespace basic_motion::controller::states
