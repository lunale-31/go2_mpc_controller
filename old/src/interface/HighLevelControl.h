#pragma once

#include "highlevel/UnitreeApi.h"

namespace interface {
    class HighLevelControl : highlevel::UnitreeApi {
    public:
        /**
         * Constructor.
         * @param node A reference to a ROS2 node used to interface with the robot.
         */
        explicit HighLevelControl(const rclcpp::Node::SharedPtr &node);

        /**
         * Requests to immediately stop all ongoing motion commands and release all joints.
         * Beware that this will make the robot drop down uncontrolledly.
         * @returns A future containing whether the command was executed successfully.
         */
        std::future<bool> damp();

        /**
         * Request to stop ongoing motions and lock joints in place.
         * @returns A future containing whether the command was executed successfully.
         */
        std::future<bool> stop_move();

        /**
         * Requests the robot to stand up.
         * @returns A future containing whether the command was executed successfully.
         */
        std::future<bool> stand_up();

        /**
         * Requests the robot to sit down.
         * This places the robot in a position safe for turning it off.
         * @returns A future containing whether the command was executed successfully.
         */
        std::future<bool> sit_down();

        /**
         * Requests the robot to walk at the given velocities.
         * @param vx X (forwards) velocity in meters per second. Must be in the range [-2.5, 3.8].
         * @param vy Y (sidewards) velocity in meters per second. Must be in the range [-1.0, 1.0].
         * @param vyaw Yaw (angular) velocity in radians per second. Must be in the range [-4.0, 4.0].
         * @returns A future containing whether the command was executed successfully.
         */
        std::future<bool> move(float vx, float vy, float vyaw);

        /**
         * Requests the robot to move into a given posture.
         * @param roll Roll angle in radians. Must be in the range [-0.75, 0.75].
         * @param pitch Pitch angle in radians. Must be in the range [-0.75, 0.75].
         * @param yaw Yaw angle in radians. Must be in the range [-0.6, 0.6].
         * @returns A future containing whether the command was executed successfully.
         */
        std::future<bool> euler(float roll, float pitch, float yaw);

        using SharedPtr = std::shared_ptr<HighLevelControl>;
    };
} // namespace interface