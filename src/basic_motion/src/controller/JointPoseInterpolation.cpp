#include "JointPoseInterpolation.h"
#include <algorithm>

JointPoseInterpolation::JointPoseInterpolation(const JointPose &from, const JointPose &to, JointPose &target, float transition_time)
    : from_(from), to_(to), target_(target), transition_time_(transition_time) {
    if (transition_time < 0.0f) {
        throw std::invalid_argument("Transition time must be positive!");
    }
}

void JointPoseInterpolation::update(float dt) {
    current_time_ = std::clamp(current_time_ + dt, 0.0f, transition_time_);
    float progress = transition_time_ == 0.0f ? 1.0f : (current_time_ / transition_time_);

    const auto interpolate = [](const float from, const float to, const float progress) {
        return from + progress * (to - from);
    };

    target_.hip_q = interpolate(from_.hip_q, to_.hip_q, progress);
    target_.thigh_q = interpolate(from_.thigh_q, to_.thigh_q, progress);
    target_.calf_q = interpolate(from_.calf_q, to_.calf_q, progress);
}

bool JointPoseInterpolation::finished() {
    return current_time_ >= transition_time_;
}
