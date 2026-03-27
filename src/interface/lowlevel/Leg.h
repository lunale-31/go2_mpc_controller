#pragma once

#include "Joint.h"

namespace interface::lowlevel {
    class Leg {
    public:
        /**
         * Constructor.
         * Only to be invoked by the LowLevelControl constructor.
         */
        Leg(Joint::SharedPtr &hip, Joint::SharedPtr &thigh, Joint::SharedPtr &calf);

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

        using SharedPtr = std::shared_ptr<Leg>;

    private:
        Joint::SharedPtr &hip_;
        Joint::SharedPtr &thigh_;
        Joint::SharedPtr &calf_;
    };
} // namespace interface::lowlevel
