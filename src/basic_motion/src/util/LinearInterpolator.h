#pragma once

namespace basic_motion::util {
    /**
     * Linearly interpolates between from and to
     * @param from Start of interpolation range
     * @param to End of interpolation range
     * @param p Position in interpolation range, in [0, 1]
     */
    float linear_interpolate(const float from, const float to, const float p);

    /**
     * Linear interpolator with integrated progress state
     */
    class LinearInterpolator {
        private:
            // Interpolation range
            float from_, to_, current_;

            // Progress
            float progress_ = 0.0f, target_;

        public:
            /**
             * Constructor.
             * @param from Start of interpolation range
             * @param to End of interpolation range
             * @param target End of progress range
             */
            LinearInterpolator(float from, float to, float target = 1.0f);

            /**
             * Increments the progress state
             * @param increment How much the progress is incremented
             */
            void update(float increment);

            /**
             * Returns the interpolated value from range [from, to] 
             * given the current progress
             */
            float current();

            /**
             * Returns whether the progress has reached its target value
             */
            bool finished();
    };
} // namespace basic_motion::util
