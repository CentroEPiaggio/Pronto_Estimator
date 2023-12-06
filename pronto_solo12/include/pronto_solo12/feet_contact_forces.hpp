#pragma once

#include <pronto_quadruped_commons/feet_contact_forces.h>
#include <pronto_quadruped_commons/leg_vector_map.h>
#include "pronto_solo12/feet_jacobians.hpp"
#include "pronto_solo12/dynamics.hpp"
#include "pronto_core/definitions.hpp"

namespace pronto {
namespace solo{

class FeetContactForces : public pronto::quadruped::FeetContactForces {

public:
    typedef Eigen::Matrix<double, 19, 1> JointStatePinocchio;
    typedef Eigen::Matrix<double, 18, 1> JointVelocityPinocchio;
    
    using Vector3d = pronto::quadruped::Vector3d;
    using JointState = pronto::quadruped::JointState;
    using LegID = pronto::quadruped::LegID;
    using LegVectorMap = quadruped::LegVectorMap;

public:
    
    FeetContactForces(solo::FeetJacobians&  feet_jacs, solo::Dynamics& dynamics) :
        feet_jacs_(feet_jacs),
        dynamics_(dynamics)
    {}

    virtual ~FeetContactForces() override {}

    bool getFootGRF(const JointState& q,
                    const JointState& qd,
                    const JointState& tau,
                    const Quaterniond& orient,
                    const LegID& leg,
                    Vector3d& foot_grf,
                    const JointState& qdd = JointState::Zero(),
                    const Vector3d& xd = Vector3d::Zero(),
                    const Vector3d& xdd = Vector3d::Zero(),
                    const Vector3d& omega = Vector3d::Zero(),
                    const Vector3d& omegad = Vector3d::Zero()) override;


private:

    // Add here:
    // force transforms
    FeetJacobians feet_jacs_;
    Dynamics dynamics_;
    Eigen::Matrix3d M_leg;
    Eigen::Vector3d c_leg;

};
}
}
