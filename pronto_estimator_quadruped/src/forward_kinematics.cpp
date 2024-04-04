#include "pronto_estimator_quadruped/forward_kinematics.hpp"

namespace pronto{
namespace estimator_quad{

    typedef pronto::quadruped::Vector3d Vector3d;
    typedef pronto::quadruped::Matrix3d Matrix3d;

    Vector3d ForwardKinematics::getFootPos(const JointState& q, const LegID& leg)
    {
        std::cerr << "ERROR CHECK: Program entered wrong \"getFootPos\" function" << std::endl;
    }
    
    Matrix3d ForwardKinematics::getFootOrientation(const JointState& q, const LegID& leg)
    {
        std::cerr << "ERROR CHECK: Program entered wrong \"getFootOrientation\" function" << std::endl;
    }
    
    Vector3d ForwardKinematics::getFootPos(const JointStatePinocchio& q, const LegID& leg)
    {
        return feet_jacobian_.getFootPos(q, leg);
    }

    Matrix3d ForwardKinematics::getFootOrientation(const JointStatePinocchio& q, const LegID& leg)
    {
        return feet_jacobian_.getFootOrientation(q, leg);
    }
} // namespace estimator_quad

}