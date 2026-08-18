#pragma once

namespace robot{
    static constexpr int num_legs_ = 4;
    static constexpr int num_leg_joints_ = 3;
    static constexpr int num_motors_ = num_legs_ * num_leg_joints_;
    
    enum class Leg : int {
        FR = 0, 
        FL = 1,
        RR = 2, 
        RL = 3
    };

    enum class Joint : int {
        HIP = 0,
        THIGH = 1,
        CALF = 2
    };

    constexpr int get_motor_index(Leg leg, Joint joint) {
    return static_cast<int>(leg) * 3 + static_cast<int>(joint);
    }

    constexpr int to_int(Leg leg) {
        return static_cast<int>(leg);
    }
}