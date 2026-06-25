#pragma once

#include <eigen3/Eigen/Dense>
#include <go2_utils/kinematics.h>
#include <go2_utils/robot.h>

namespace basic_motion::controller::states::stand {
    Eigen::Vector3f pose(const float height,
                              go2_utils::robot::LegPair pair,
                              go2_utils::robot::LegSide side);
}