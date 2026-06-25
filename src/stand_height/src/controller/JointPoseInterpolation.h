#pragma once

#include "JointPose.h"

class JointPoseInterpolation {
    private:
        const JointPose from_, to_;
        JointPose &target_;
        float transition_time_, current_time_ = 0.0f;

    public:
        JointPoseInterpolation(const JointPose &from, const JointPose &to, JointPose &target, float transition_time);
        void update(float dt);
        bool finished();
};