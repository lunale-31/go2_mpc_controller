#include "JointPose.h"
#include <algorithm>

JointPose JointPose::fromLegState(const go2_utils::interface::lowlevel::Leg::SharedPtr &leg) {
    JointPose jp;
    fromLegState(jp, leg);
    return jp;
}

void JointPose::fromLegState(JointPose &jp, const go2_utils::interface::lowlevel::Leg::SharedPtr &leg) {
    jp.hip_q = leg->hip()->state().q;
    jp.thigh_q = leg->thigh()->state().q;
    jp.calf_q = leg->calf()->state().q;
}

void JointPose::toLegCommand(JointPose &jp, const go2_utils::interface::lowlevel::Leg::SharedPtr &leg) {
    leg->hip()->cmd().q = jp.hip_q;
    leg->thigh()->cmd().q = jp.thigh_q;
    leg->calf()->cmd().q = jp.calf_q;
}

JointPose JointPose::interpolate(const JointPose &jp_a, const JointPose &jp_b, const float ratio) {
    const float ratio_norm = std::clamp(ratio, 0.0f, 1.0f);
    JointPose jp;
    jp.hip_q = ratio_norm * jp_a.hip_q + (1.0f - ratio_norm) * jp_b.hip_q;
    jp.thigh_q = ratio_norm * jp_a.thigh_q + (1.0f - ratio_norm) * jp_b.thigh_q;
    jp.calf_q = ratio_norm * jp_a.calf_q + (1.0f - ratio_norm) * jp_b.calf_q;
    return jp;
}
