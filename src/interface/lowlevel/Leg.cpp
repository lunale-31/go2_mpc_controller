#include "Leg.h"

namespace interface::lowlevel {
    Motor &Leg::hip() const {
        return hip_;
    }

    Motor &Leg::thigh() const {
        return thigh_;
    }

    Motor &Leg::calf() const {
        return calf_;
    }

    Leg::Leg(Motor &hip, Motor &thigh, Motor &calf) : hip_(hip), thigh_(thigh), calf_(calf) {
        // empty
    }
} // namespace interface::lowlevel
