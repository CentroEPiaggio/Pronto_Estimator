#pragma once

#include <pronto_quadruped_commons/feet_jacobians.h>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace pronto {
namespace solo{

class FeetJacobians : public pronto::quadruped::FeetJacobians {
public:

    typedef pronto::quadruped::FootJac FootJac;
    typedef pronto::quadruped::JointState JointState;
    typedef Eigen::Matrix<double, 19, 1> JointStatePinocchio;
    typedef Eigen::Matrix<double, 18, 1> JointVelocityPinocchio;
    typedef pronto::quadruped::LegID LegID;
    typedef pronto::quadruped::Vector3d Vector3d;   
    typedef pronto::quadruped::Matrix3d Matrix3d; 

    FeetJacobians(pinocchio::Model & model, pinocchio::Data & data);

    virtual ~FeetJacobians() override {};

    void updateConfiguration(const JointStatePinocchio& q);
    
    pinocchio::Data::Matrix6x ComputeJacobian(const JointStatePinocchio& q, const LegID& leg);

    FootJac getFootJacobian(const JointStatePinocchio& q, const LegID& leg) ;
    FootJac getFootJacobianAngular(const JointStatePinocchio& q, const LegID& leg); // is this function used?

    // Redeclaring these function to see if some library get into them
    FootJac getFootJacobian(const JointState& q, const LegID& leg) override;
    FootJac getFootJacobianAngular(const JointState& q, const LegID& leg) override;

    Vector3d getFootPos(const JointStatePinocchio& q, const LegID& leg);
    Matrix3d getFootOrientation(const JointStatePinocchio& q, const LegID& leg); 
        

private:
    
    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::FrameIndex leg_id;
    FootJac jacobian;
    Eigen::Matrix<double, 6, 18> J;
    // pinocchio::Data::Matrix6x J;
    pinocchio::SE3 T;
    JointStatePinocchio prev_q;
    
};

}  // namespace solo
}  // namespace pronto 

