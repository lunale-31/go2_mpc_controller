#pragma once

#include <cstdint>

namespace interface::lowlevel {

    /**
     * IMU State measured by the robot
     */
    struct ImuState {
        // Trunk pose in quarternion representation
        float quarternion[4];

        // Angular pose of the robot's trunk
        float roll, pitch, yaw;

        // Angular velocity of the robot's trunk
        float roll_velo, pitch_velo, yaw_velo;

        // Acceleration of the robot's trunk
        float x_acc, y_acc, z_acc;

        // IMU temperature
        uint8_t temperature;

        using SharedPtr = std::shared_ptr<ImuState>;
    };
} // namespace interface::lowlevel