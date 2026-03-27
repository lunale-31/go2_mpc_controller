#include "Leg.h"

namespace interface::lowlevel {
    Joint &Leg::hip() const {
        return hip_;
    }

    Joint &Leg::thigh() const {
        return thigh_;
    }

    Joint &Leg::calf() const {
        return calf_;
    }

    Leg::Leg(Joint &hip, Joint &thigh, Joint &calf) : hip_(hip), thigh_(thigh), calf_(calf) {
        // empty
    }
} // namespace interface::lowlevel
