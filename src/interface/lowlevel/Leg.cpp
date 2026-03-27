#include "Leg.h"

namespace interface::lowlevel {
    Joint::SharedPtr &Leg::hip() const {
        return hip_;
    }

    Joint::SharedPtr &Leg::thigh() const {
        return thigh_;
    }

    Joint::SharedPtr &Leg::calf() const {
        return calf_;
    }

    Leg::Leg(Joint::SharedPtr &hip, Joint::SharedPtr &thigh, Joint::SharedPtr &calf) : hip_(hip), thigh_(thigh), calf_(calf) {
        // empty
    }
} // namespace interface::lowlevel
