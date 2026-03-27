#pragma once

#include "Joint.h"

namespace interface::lowlevel {
    class Leg {
    public:
        /**
         * Constructor.
         * Only to be invoked by the LowLevelControl constructor.
         */
        Leg(Joint &hip, Joint &thigh, Joint &calf);

        /**
         * Gets the hip (top-most) joint.
         */
        Joint &hip() const;

        /**
         * Gets the thigh (middle) joint.
         */
        Joint &thigh() const;

        /**
         * Gets the calf (lower-most) joint.
         */
        Joint &calf() const;

    private:
        Joint &hip_;
        Joint &thigh_;
        Joint &calf_;
    };
} // namespace interface::lowlevel
