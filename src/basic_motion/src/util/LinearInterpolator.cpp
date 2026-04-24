#include "LinearInterpolator.h"
#include <algorithm>

namespace basic_motion::util {
    float linear_interpolate(const float from, const float to, const float p) {
        const float p_norm = std::clamp(p, 0.0f, 1.0f);
        return from + p_norm * (to - from);
    }

    LinearInterpolator::LinearInterpolator(float from, float to, float target)
        : from_(from), to_(to), current_(from), target_(target) {
        // empty
    }

    void LinearInterpolator::update(float increment) {
        progress_ = std::clamp(progress_ + increment, 0.0f, target_);
        current_ = linear_interpolate(from_, to_, target_ == 0.0f ? 1.0f : progress_ / target_);
    }

    float LinearInterpolator::current() {
        return current_;
    }
    
    bool LinearInterpolator::finished() {
        return progress_ >= target_;
    }
} // namespace basic_motion::util
