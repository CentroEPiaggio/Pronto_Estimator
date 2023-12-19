#pragma once

#include "pronto_estimator_quadruped/feet_jacobians.hpp"

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace pronto{
namespace estimator_quad{

class Dynamics{
public:
    // typedef pronto::quadruped::JointState JointState;
    typedef Eigen::Matrix<double, 19, 1> JointStatePinocchio;
    typedef Eigen::Matrix<double, 18, 1> JointVelocityPinocchio;
    typedef pronto::quadruped::LegID LegID;
    typedef pronto::quadruped::Vector3d Vector3d; 

    Dynamics(pinocchio::Model & model, pinocchio::Data & data);

    virtual ~Dynamics() {};

    void updateDynamics(const JointStatePinocchio& q, const JointVelocityPinocchio& qd, const JointVelocityPinocchio& qdd);
    Eigen::Matrix3d getInertiaMatrix(const JointStatePinocchio& q, const JointVelocityPinocchio& qd, const JointVelocityPinocchio& qdd, const LegID& leg);
    Vector3d getNonLinear(const JointStatePinocchio& q, const JointVelocityPinocchio& qd, const JointVelocityPinocchio& qdd, const LegID& leg);  
    Vector3d getRNEA(const JointStatePinocchio& q, const JointVelocityPinocchio& qd, const JointVelocityPinocchio& qdd, const LegID& leg);


private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    JointStatePinocchio prev_q;

};

}  // namespace estimator_quad
}  // namespace pronto 