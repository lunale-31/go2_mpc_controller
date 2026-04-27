#include <go2_utils/interface/lowlevel/Leg.h>
#include <go2_utils/kinematics.h>
#include <iostream>

namespace go2_utils::interface::lowlevel {
    Leg::Leg(Joint::SharedPtr &hip, Joint::SharedPtr &thigh, Joint::SharedPtr &calf,
             robot::LegPair pair, robot::LegSide side)
        : hip_(hip), thigh_(thigh), calf_(calf), side_(side), pair_(pair) {
        // empty
    }

    Joint::SharedPtr &Leg::hip() const {
        return hip_;
    }

    Joint::SharedPtr &Leg::thigh() const {
        return thigh_;
    }

    Joint::SharedPtr &Leg::calf() const {
        return calf_;
    }

    robot::LegSide Leg::side() const {
        return side_;
    }

    robot::LegPair Leg::pair() const {
        return pair_;
    }

    void Leg::command_joint_angles(Eigen::Vector3f &joints) const {
        hip_->cmd().q = joints.x();
        thigh_->cmd().q = joints.y();
        calf_->cmd().q = joints.z();
    }

    Eigen::Vector3f Leg::joint_angles() const {
        return Eigen::Vector3f(hip_->state().q,
                               thigh_->state().q,
                               calf_->state().q);
    }

    Eigen::Vector3f Leg::foot_position() const {
        auto pos_opt = kinematics::forwards(joint_angles(), side_);
        if (!pos_opt) {
            std::cerr << joint_angles() << std::endl;
            throw std::invalid_argument("Could not compute foot position.");
        }
        return *pos_opt;
    }
} // namespace go2_utils::interface::lowlevel
