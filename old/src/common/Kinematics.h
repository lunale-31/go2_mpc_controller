#pragma once

#include <eigen3/Eigen/Dense>
#include <optional>

namespace common
{
    // Define limb lengths (in meters)
    const float L_1 = 0.095f, L_2 = 0.213f, L_3 = 0.230f;

    // Define hip joint limits (in radians)
    const float theta_1_min = -0.85f, theta_1_max = 0.85f;

    // Define thigh joint limits (in radians)
    const float theta_2_min = -0.58f, theta_2_max = 4.6f;

    // Define calf joint limits (in radians)
    const float theta_3_min = -2.8f, theta_3_max = -0.95f;
    
    /**
     * The side on which a robot leg is mounted.
     * Left and right are defined by looking at the robot from the top and having the LiDAR sensor face up.
     */
    enum LegSide {
        LEFT,
        RIGHT,
    };

    /**
     * Computes the location reached by a foot given the joint angle configuration.
     * @param joints The joint angle configuration (hip, thigh, calf)'
     * @param side The side the leg is mounted on
     * @returns The position vector reached by the foot, or None if the joint configuration is invalid.
     */
    std::optional<Eigen::Vector3f> forwards_kinematics(const Eigen::Vector3f &joints, LegSide side);

    /**
     * Computes the joint angle configuration required to reach a given target point.  
     * @param target The target position to be reached
     * @param side The side the leg is mounted on
     * @returns The joint angle configuration(s) (hip, thigh, calf)'. Empty, if the position is unreachable.
     */
    std::vector<Eigen::Vector3f> inverse_kinematics(const Eigen::Vector3f &target, LegSide side);
} // namespace common
