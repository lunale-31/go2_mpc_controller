#pragma once

#include <go2_utils/interact/LowLevelControl.h>
#include <map>

class LegController {
public:
    enum FootPosition : int {
        FORWARDS = 0,
        MIDDLE = 1,
        BACKWARDS = 2
    };

private:
    /// the leg we control
    go2_utils::interact::lowlevel::Leg::SharedPtr leg_;

    /// the step of the walking cycle we're currently in
    int swing_step_, current_step_ = 0;

    /// the total number of steps
    static constexpr int count_step_ = 6;

    /// the step indices in which the robot pushes forwards (all legs on the ground)
    static constexpr int push_steps[] = {2, 5};

    /// the progress within the current step, bound in [0.0, 1.0]
    float step_progress_ = 0.0f;

    /// the number of ticks of a step
    /// We use a float here to prevent converting the number in every computation,
    /// even though it is a integer value.
    static constexpr float step_length_ = 100.0f;

    /// relative foot positions at the start and end of the current step
    FootPosition step_start_, step_end_;
    void advance_step_boundaries();

    float x_positions(FootPosition pos) {
        bool front = leg_->pair() == go2_utils::robot::FRONT;
        switch (pos) {
            case FORWARDS:
                return front ? 0.2f : 0.1f;
            case MIDDLE:
                return front ? 0.05f : -0.05f;
            case BACKWARDS:
                return front ? -0.1f : -0.2f;
        }
        throw std::runtime_error("Should not reach this.");
    }
    static constexpr float z_stand = -0.3, z_swing = -0.15;

    Eigen::Vector3f joint_angles_for_foot_position(const float x, const float z);

    inline bool current_step_is_push();

    void perform_swing();
    void perform_push();
    void perform_stand();

    void leg_set_kps(float kp);

public:
    LegController(go2_utils::interact::lowlevel::Leg::SharedPtr &leg, int swing_step, FootPosition first_step_start, FootPosition first_step_end);

    void tick();
};