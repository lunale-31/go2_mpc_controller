#include "JointPose.h"

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
