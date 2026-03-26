#pragma once

#include "Motor.h"

namespace interface::lowlevel {
    class Leg {
    public:
        Leg(Motor &hip, Motor &thigh, Motor &calf);

        Motor &hip() const;

        Motor &thigh() const;

        Motor &calf() const;

    private:
        Motor &hip_;
        Motor &thigh_;
        Motor &calf_;
    };
} // namespace interface::lowlevel
