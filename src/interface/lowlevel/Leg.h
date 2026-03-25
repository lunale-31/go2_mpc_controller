//
// Created by ubuntu on 3/23/26.
//

#ifndef RTSC_UNITREE_ROS2_LEG_H
#define RTSC_UNITREE_ROS2_LEG_H

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
}

#endif //RTSC_UNITREE_ROS2_LEG_H
