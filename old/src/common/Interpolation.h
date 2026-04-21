#pragma once

#include <cmath>
#include <algorithm>

namespace common {
    /**
     * Returns the linear interpolation in the range [0, 1]
     */
    constexpr inline double interpolate_linear(double pos) {
        return std::clamp(0.0, pos, 1.0);
    }

    /**
     * Returns the linear interpolation in the range [from, to]
     */
    constexpr inline double interpolate_linear_between(double pos, double from, double to) {
        return from + interpolate_linear(pos) * (to - from);
    }

    /**
     * Returns the square interpolation in the range [0, 1]
     */
    constexpr inline double interpolate_square(double pos) {
        if (pos <= 0.0) {
            return 0.0;
        } else if (pos >= 1.0) {
            return 1.0;
        } else if (pos < 0.5) {
            return 2.0 * pos * pos;
        } else { // 0.5 <= pos < 1.0
            // This is equivalent to 1 - (sqrt(2) * (x - 1))^2
            return -2.0 * pos * pos + 4 * pos - 1.0;
        }
    }

    /**
     * Returns the square interpolation in the range [from, to]
     */
    constexpr inline double interpolate_square_between(double pos, double from, double to) {
        return from + interpolate_square(pos) * (to - from);
    }
} // namespace common
