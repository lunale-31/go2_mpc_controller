#include "Kinematics.h"

#include <cmath>

namespace common {
    // Define limb lengths (in meters)
    static const float L_1 = 0.095f, L_2 = 0.213f, L_3 = 0.230f;

    // Define hip joint limits (in radians)
    static const float theta_1_min = -0.85f, theta_1_max = 0.85f;

    // Define thigh joint limits (in radians)
    static const float theta_2_min = -0.58f, theta_2_max = 4.6f;

    // Define calf joint limits (in radians)
    static const float theta_3_min = -2.8f, theta_3_max = -0.95f;

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

        const float z = L_1_adapted * s_1 - (L_2_c_2 + L_3_c_23) * c_1;
        if (std::isnan(z)) {
            return std::nullopt;
        }

        return Eigen::Vector3f(x, y, z);
    }

    /**
     * Clamps values that are less than an epsilon outside the range [lower, upper] into that range.
     * Other values remain untouched.
     */
    static float epsilon_clamp(float val, const float lower, const float upper) {
        if (lower - epsilon < val && val < lower) {
            return lower;
        } else if (upper + epsilon > val && val > upper) {
            return upper;
        }
        return val;
    }

    /**
     * Computes the calf joint angle for a desired target position, given the hip joint angle.
     * @param target The desired target point p
     * @param side The side the leg is mounted on
     * @param theta_1 The already-known hip joint angle
     * @returns The angles of the thigh and calf joints, if a solution exists, otherwise None
     */
    static std::optional<std::pair<float, float>> compute_calf(const Eigen::Vector3f &target, LegSide side, const float theta_1) {
        const float &p_x = target.x(), &p_y = target.y(), &p_z = target.z();

        // printf("p = (%.4f, %.4f, %.4f)\n", p_x, p_y, p_z);

        (void)side;

        const Eigen::Vector3f d_vec(p_x, p_y - L_1 * cosf(theta_1), p_z - L_1 * sinf(theta_1));

        const float d_len_squared = abs(pow(d_vec.x(), 2.0f) + pow(d_vec.y(), 2.0f) + pow(d_vec.z(), 2.0f));
        // printf("L_1 * c_1 = %.4f\n", L_1 * cosf(theta_1));
        // printf("L_1 * s_1 = %.4f\n", L_1 * sinf(theta_1));
        // printf("d = (%.4f, %.4f, %.4f)\n", d_vec.x(), d_vec.y(), d_vec.z());
        // printf("d_len = %.4f\n", sqrtf(d_len_squared));

        const float theta_3 = -acosf((powf(L_2, 2.0f) + powf(L_3, 2.0f) - d_len_squared) / (-2.0f * L_2 * L_3));
        if (joint_valid(theta_3, theta_3_min, theta_3_max)) {
            // printf("theta_3 = %.4f is possible.\n", theta_3);

            // We found a solution for theta_3, so compute theta_2 now.
            const float &p_x = target.x();
            const float d_len = sqrtf(d_len_squared);

            const float alpha = acosf(
                (d_len_squared + powf(L_2, 2.0f) - powf(L_3, 2.0f)) / (2.0f * d_len * L_2));
            // printf("alpha = %.4f\n", alpha);

            // We have to clamp here due to fp imprecision as we sometimes get values barely outside the range [-1, 1].
            const float beta = acosf(
                epsilon_clamp(((d_vec.y() * sinf(theta_1) - d_vec.z() * cosf(theta_1)) / d_len), -1.0f, 1.0f));
            // printf("d_y * s_1 = %.4f\n", d_vec.y() * sinf(theta_1));
            // printf("d_z * c_1 = %.4f\n", d_vec.z() * cosf(theta_1));
            // printf("(d_y * s_1 - d_z * c_1) / |d| = %.4f\n", (d_vec.y() * sinf(theta_1) - d_vec.z() * cosf(theta_1)) / d_len);
            // printf("beta = %.4f\n", beta);

            float theta_2 = p_x >= 0.0 ? alpha + beta : alpha - beta;

            if (joint_valid(theta_2, theta_2_min, theta_2_max)) {
                // printf("theta_2 = %.4f is possible.\n", theta_2);
                return std::pair<float, float>(theta_2, theta_3);
            } else if (joint_valid(2.0f * M_PIf + theta_2, theta_2_min, theta_2_max)) {
                return std::pair<float, float>(2.0f * M_PIf + theta_2, theta_3);
            }

            // printf("theta_2 = %.4f is invalid.\n", theta_2);
        } else {
            // TODO: remove else branch
            // printf("theta_3 = %.4f is invalid.\n", theta_3);
        }
        return std::nullopt;
    }

    std::vector<Eigen::Vector3f> inverse_kinematics(const Eigen::Vector3f &target, LegSide side) {
        const float &p_y = target.y(), &p_z = target.z();

        const float R = sqrtf(powf(p_y, 2.0f) + powf(p_z, 2.0f));
        const float alpha = acosf(epsilon_clamp(p_y / R, -1.0f, 1.0f));
        // printf("alpha = %.4f\n", alpha);

        const float beta = acosf(epsilon_clamp(L_1 / R, -1.0f, 1.0f));
        // printf("beta = %.4f\n", beta);

        std::vector<Eigen::Vector3f> result;

        // check both solutions
        const float theta_1_1 = p_z >= 0.0f ? alpha + beta : -(alpha + beta);
        if (joint_valid(theta_1_1, theta_1_min, theta_1_max)) {
            // printf("theta_1 = %.4f is possible.\n", theta_1_1);
            if (auto theta_23 = compute_calf(target, side, theta_1_1); theta_23) {
                result.emplace_back(Eigen::Vector3f(theta_1_1, theta_23->first, theta_23->second));
            }
        } else {
            // printf("theta_1 = %.4f is invalid.\n", theta_1_1);
        }

        const float theta_1_2 = p_z >= 0.0f ? alpha - beta : beta - alpha;
        if (joint_valid(theta_1_2, theta_1_min, theta_1_max)) {
            // printf("theta_1 = %.4f is possible.\n", theta_1_2);
            if (auto theta_23 = compute_calf(target, side, theta_1_2); theta_23) {
                result.emplace_back(Eigen::Vector3f(theta_1_2, theta_23->first, theta_23->second));
            }
        } else {
            // printf("theta_1 = %.4f is invalid.\n", theta_1_2);
        }

        if (result.empty()) {
            // printf("    Considered the solutions %.4f and %.4f.\n", theta_1_1, theta_1_2);
        }

        // printf("====\n");

        return result;
    }
} // namespace common
