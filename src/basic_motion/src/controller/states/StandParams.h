#pragma once

namespace basic_motion::controller::states {
    struct StandParams {
        // The height of the body above the ground
        float body_height;

        // The time to transition to the height (in seconds)
        float transition_time;
    };
}
