#pragma once

namespace basic_motion::controller::states {
    struct GaitParams {
        // The height of the body above the ground
        float body_height;

        // The height to which the leg is pulled when swinging
        float swing_height;

        // The leg displacement in negative x direction when gaiting
        float swing_min;

        // The leg displacement in positive x direction when gaiting
        float swing_max;

        // The time to transition the parameters
        float transition_time = 2.0f /* seconds */;
    };
}
