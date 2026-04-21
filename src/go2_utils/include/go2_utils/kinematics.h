#pragma once

#include "robot.h"
#include <eigen3/Eigen/Dense>
#include <optional>

namespace go2_utils::kinematics {
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
    std::optional<Eigen::Vector3f> forwards(const Eigen::Vector3f &joints, LegSide side);

    /**
     * Computes the joint angle configuration required to reach a given target point.
     * @param target The target position to be reached
     * @param side The side the leg is mounted on
     * @returns The joint angle configuration(s) (hip, thigh, calf)'. Empty, if the position is unreachable.
     */
    std::vector<Eigen::Vector3f> inverse(const Eigen::Vector3f &target, LegSide side);

} // namespace go2_utils::kinematics
