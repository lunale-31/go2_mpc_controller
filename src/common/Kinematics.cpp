#include "Kinematics.h"

#include <cmath>

namespace common {
    // Define limb lengths (in meters)
    static const float L_1 = 0.095f, L_2 = 0.213f, L_3 = 0.230f;

    // Define hip joint limits (in radians)
    static const float theta_1_min = -0.75f, theta_1_max = 0.75f;

    // Define thigh joint limits (in radians)
    static const float theta_2_min = -1.55f, theta_2_max = 3.53f;

    // Define calf joint limits (in radians)
    static const float theta_3_min = -2.80f, theta_3_max = -0.92f;

    // Define tolerable computation error
    static const float epsilon = 0.001f;

    /**
     * Check if a given joint angle is valid for that joint
     */
    static inline bool joint_valid(const float joint, const float min, const float max) {
        return !std::isnan(joint) && (min - epsilon) <= joint && joint <= (max + epsilon);
    }

    /**
     * Check if a vector of joint angles is valid
     */
    static bool joints_valid(const Eigen::Vector3f &joints) {
        const float theta_1 = joints.x(), theta_2 = joints.y(), theta_3 = joints.z();
        return joint_valid(theta_1, theta_1_min, theta_1_max) && joint_valid(theta_2, theta_2_min, theta_2_max) && joint_valid(theta_3, theta_3_min, theta_3_max);
    }

    std::optional<Eigen::Vector3f> forwards_kinematics(const Eigen::Vector3f &joints, LegSide side) {
        if (!joints_valid(joints)) {
            return std::nullopt;
        }

        const float theta_1 = joints.x(), theta_2 = joints.y(), theta_3 = joints.z();

        const float s_1 = sinf(theta_1), c_1 = cosf(theta_1);
        const float s_2 = sinf(theta_2), c_2 = cosf(theta_2);

        // Adapt L_1 to account for inverted hip direction on the right side
        const float L_1_adapted = side == LegSide::LEFT ? L_1 : -L_1;

        const float x = L_2 * s_2 + L_3 * sin(theta_2 + theta_3);
        if (std::isnan(x)) {
            return std::nullopt;
        }

        // precompute reused components
        const float L_2_c_2 = L_2 * c_2;
        const float L_3_c_23 = L_3 * cos(theta_2 + theta_3);

        const float y = L_1_adapted * c_1 + (L_2_c_2 + L_3_c_23) * s_1;
        if (std::isnan(y)) {
            return std::nullopt;
        }

        const float z = L_1_adapted * s_1 + (L_2_c_2 + L_3_c_23) * c_1;
        if (std::isnan(z)) {
            return std::nullopt;
        }

        return Eigen::Vector3f(x, y, z);
    }

    std::optional<Eigen::Vector3f> inverse_kinematics(const Eigen::Vector3f &target, LegSide side) {
        return std::nullopt;
    }
} // namespace common
