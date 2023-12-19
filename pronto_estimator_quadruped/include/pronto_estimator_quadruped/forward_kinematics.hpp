#pragma once

#include <pronto_quadruped_commons/forward_kinematics.h>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include "pronto_estimator_quadruped/feet_jacobians.hpp"


namespace pronto {
namespace estimator_quad{

// this class is necessary for pronto but all the computations are made by FeetJacobians
class ForwardKinematics : public pronto::quadruped::ForwardKinematics {
public:
    typedef pronto::quadruped::JointState JointState;
    typedef Eigen::Matrix<double,19,1> JointStatePinocchio;
    typedef pronto::quadruped::LegID LegID;
    typedef pronto::quadruped::Vector3d Vector3d; 
    typedef pronto::quadruped::Matrix3d Matrix3d;  

    ForwardKinematics(pronto::estimator_quad::FeetJacobians& feet_jacobian) :
    feet_jacobian_(feet_jacobian)
    {}

    Vector3d getFootPos(const JointState& q, const LegID& leg) override;
    Matrix3d getFootOrientation(const JointState& q, const LegID& leg) override; 

    Vector3d getFootPos(const JointStatePinocchio& q, const LegID& leg);
    Matrix3d getFootOrientation(const JointStatePinocchio& q, const LegID& leg); 


private:

    pronto::estimator_quad::FeetJacobians feet_jacobian_;

};

}
}