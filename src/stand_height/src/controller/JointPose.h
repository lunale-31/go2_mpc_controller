#pragma once

#include <go2_utils/interact/lowlevel/Leg.h>

struct JointPose {
    float hip_q = 0.0f;
    float thigh_q = 1.52f;
    float calf_q = -2.65f;

    static JointPose fromLegState(const go2_utils::interact::lowlevel::Leg::SharedPtr &leg);
    static void fromLegState(JointPose &jp, const go2_utils::interact::lowlevel::Leg::SharedPtr &leg);
    static void toLegCommand(JointPose &jp, const go2_utils::interact::lowlevel::Leg::SharedPtr &leg);
    static JointPose interpolate(const JointPose &jp_a, const JointPose &jp_b, const float ratio);
};