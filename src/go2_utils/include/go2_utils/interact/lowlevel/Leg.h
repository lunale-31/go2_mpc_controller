#pragma once

#include "Joint.h"
#include <eigen3/Eigen/Dense>
#include <go2_utils/robot.h>

namespace go2_utils::interact::lowlevel {
    class Leg {
    public:
        /**
         * Constructor.
         * Only to be invoked by the LowLevelControl constructor.
         */
        Leg(Joint::SharedPtr &hip, Joint::SharedPtr &thigh, Joint::SharedPtr &calf, 
            std::shared_ptr<uint16_t> &foot_force, robot::LegPair pair, robot::LegSide side);

        /**
         * Gets the hip (top-most) joint.
         */
        Joint::SharedPtr &hip() const;

        /**
         * Gets the thigh (middle) joint.
         */
        Joint::SharedPtr &thigh() const;

        /**
         * Gets the calf (lower-most) joint.
         */
        Joint::SharedPtr &calf() const;

        /**
         * Sets the angles (q) in the joint commands according to given vector.
         */
        void command_joint_angles(const Eigen::Vector3f &joints) const;

        /**
         * Gets the joint angles of the leg.
         */
        Eigen::Vector3f joint_angles() const;

        /**
         * Gets the foot position of the leg.
         */
        Eigen::Vector3f foot_position() const;

        /**
         * Gets the side of the robot onto which the leg is mounted.
         */
        robot::LegSide side() const;

        /**
         * Gets the pair of legs to which the leg belongs.
         */
        robot::LegPair pair() const;

        /**
         * Gets the force measured at the foot.
         */
        uint16_t foot_force() const;

        using SharedPtr = std::shared_ptr<Leg>;

    private:
        Joint::SharedPtr &hip_;
        Joint::SharedPtr &thigh_;
        Joint::SharedPtr &calf_;

        std::shared_ptr<uint16_t> foot_force_;

        robot::LegSide side_;
        robot::LegPair pair_;
    };
} // namespace interface::lowlevel
