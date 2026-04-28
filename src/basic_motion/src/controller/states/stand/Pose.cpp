#include "Pose.h"

// Constants
static constexpr float sit_down_threshold = 0.12f;
static constexpr float thigh_min = 0.0f, thigh_max = 2.5f;
static const auto sit_down_pose_front_left = Eigen::Vector3f(-0.05f, 1.3f, -2.8f);
static const auto sit_down_pose_back_left = Eigen::Vector3f(0.3f, 1.31f, -2.81f);

namespace basic_motion::controller::states::stand {
    Eigen::Vector3f pose(const float height,
                               const go2_utils::robot::LegPair pair,
                               const go2_utils::robot::LegSide side) {
        using namespace go2_utils::robot;
        if (height >= sit_down_threshold) {
            // Height above sitting threshold requested, use normal height computation
            auto joint_configurations = go2_utils::kinematics::inverse(
                Eigen::Vector3f(0.0f, side == LegSide::LEFT ? L_1 : -L_1, -height), side);
            for (auto &conf : joint_configurations) {
                if (conf.y() >= thigh_min && conf.y() <= thigh_max) {
                    return conf;
                }
            }

            // For reachable heights, this is never reached
            throw std::invalid_argument("Could not find a valid leg configuration for this height.");
        } else {
            // Height below sitting threshold requested, interpolate with floor position
            auto joint_configurations = go2_utils::kinematics::inverse(
                Eigen::Vector3f(0.0f, L_1, -sit_down_threshold), LegSide::LEFT); // we correct for the side in a few lines
            for (auto &t_conf : joint_configurations) {
                if (t_conf.y() >= thigh_min && t_conf.y() <= thigh_max) {
                    const float p = height / sit_down_threshold;
                    auto conf = t_conf * p + (pair == LegPair::FRONT ? sit_down_pose_front_left : sit_down_pose_back_left) * (1.0f - p);
                    if (side == LegSide::RIGHT) {
                        return Eigen::Vector3f(-conf.x(), conf.y(), conf.z());
                    }
                    return conf;
                }
            }

            // this is never reached
            throw std::invalid_argument("Could not find a valid leg configuration for sitting down.");
        }
    }
} // namespace basic_motion::controller::states::stand
