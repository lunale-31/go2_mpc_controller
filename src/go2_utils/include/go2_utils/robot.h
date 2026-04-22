#pragma once

namespace go2_utils::robot {
    // Leg indices
    constexpr unsigned
        LEG_FRONT_RIGHT = 0U,
        LEG_FRONT_LEFT = 1U,
        LEG_BACK_RIGHT = 2U,
        LEG_BACK_LEFT = 3U,
        LEG_MAX = LEG_BACK_LEFT,
        LEG_COUNT = LEG_MAX + 1;

    // Joint offsets within legs
    constexpr unsigned
        OFFSET_HIP = 0U,
        OFFSET_THIGH = 1U,
        OFFSET_CALF = 2U,
        JOINTS_PER_LEG = 3U;

    // Joint indices as used in low-level communication
    constexpr unsigned
        JOINT_FR_HIP = (LEG_FRONT_RIGHT * JOINTS_PER_LEG) + OFFSET_HIP,
        JOINT_FR_THIGH = (LEG_FRONT_RIGHT * JOINTS_PER_LEG) + OFFSET_THIGH,
        JOINT_FR_CALF = (LEG_FRONT_RIGHT * JOINTS_PER_LEG) + OFFSET_CALF,
        JOINT_FL_HIP = (LEG_FRONT_LEFT * JOINTS_PER_LEG) + OFFSET_HIP,
        JOINT_FL_THIGH = (LEG_FRONT_LEFT * JOINTS_PER_LEG) + OFFSET_THIGH,
        JOINT_FL_CALF = (LEG_FRONT_LEFT * JOINTS_PER_LEG) + OFFSET_CALF,
        JOINT_BR_HIP = (LEG_BACK_RIGHT * JOINTS_PER_LEG) + OFFSET_HIP,
        JOINT_BR_THIGH = (LEG_BACK_RIGHT * JOINTS_PER_LEG) + OFFSET_THIGH,
        JOINT_BR_CALF = (LEG_BACK_RIGHT * JOINTS_PER_LEG) + OFFSET_CALF,
        JOINT_BL_HIP = (LEG_BACK_LEFT * JOINTS_PER_LEG) + OFFSET_HIP,
        JOINT_BL_THIGH = (LEG_BACK_LEFT * JOINTS_PER_LEG) + OFFSET_THIGH,
        JOINT_BL_CALF = (LEG_BACK_LEFT * JOINTS_PER_LEG) + OFFSET_CALF,
        JOINT_MAX = JOINT_BL_CALF,
        JOINT_COUNT = JOINT_MAX + 1;

    // Limb lengths of the legs (hip, thigh, calf)
    constexpr float L_1 = 0.095f, L_2 = 0.213f, L_3 = 0.230f;

    // Hip joint angle limits (in radians)
    const float THETA_1_MIN = -0.85f, THETA_1_MAX = 0.85f;

    // Thigh joint angle limits (in radians)
    const float THETA_2_MIN = -0.58f, THETA_2_MAX = 4.6f;

    // Calf joint angle limits (in radians)
    const float THETA_3_MIN = -2.8f, THETA_3_MAX = -0.95f;
} // namespace go2_utils::robot
