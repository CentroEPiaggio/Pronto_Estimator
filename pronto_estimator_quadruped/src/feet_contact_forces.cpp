#include <pronto_estimator_quadruped/feet_contact_forces.hpp>
#include <pronto_quadruped_commons/geometry/rotations.h>
#include <pronto_quadruped_commons/joint_id_tricks.h>
#include "pronto_estimator_quadruped/feet_contact_forces.hpp"

using namespace iit;  //rbd library

namespace pronto {
namespace estimator_quad{
    
    bool FeetContactForces::getFootGRF(const JointState &q,
                                        const JointState &qd,
                                        const JointState &tau,
                                        const Quaterniond &orient,
                                        const LegID &leg,
                                        Vector3d &foot_grf,
                                        const JointState &qdd,
                                        const Vector3d &xd,
                                        const Vector3d &xdd,
                                        const Vector3d &omega,
                                        const Vector3d &omegad) {

        rbd::Vector6D gravity_world = rbd::Vector6D::Zero();
        gravity_world(rbd::LZ) = -rbd::g;

        rbd::Vector6D gravity_base = rbd::Vector6D::Zero();
        rbd::Vector6D base_acceleration = rbd::Vector6D::Zero();
        rbd::VelocityVector base_twist = rbd::Vector6D::Zero();

        Eigen::Matrix3d R = pronto::commons::quatToRotMat(orient);

        gravity_base.segment(rbd::LX, 3) = R * gravity_world.segment(rbd::LX, 3);

        base_acceleration.segment(rbd::LX, 3) = xdd; //this is the absolute accel of the trunk without the gravity!!!
        base_acceleration.segment(rbd::AX, 3) = omegad;

        base_twist.segment(rbd::AX, 3) = omega;
        base_twist.segment(rbd::LX, 3) = xd;

        // rbd::ForceVector h_base;
        // JointState  h_joints;

        // From now on we use pinocchio

        JointStatePinocchio qP = JointStatePinocchio::Zero();
        qP.block<4,1>(3,0) = orient.coeffs();
        qP.block<12,1>(7,0) = q;

        JointVelocityPinocchio qdP = JointVelocityPinocchio::Zero();
        qdP.block<3,1>(0,0) = xd;
        qdP.block<3,1>(3,0) = omega;
        qdP.block<12,1>(6,0) = qd;

        JointVelocityPinocchio qddP;
        qddP.head(3) = xdd;
        qddP.block<3,1>(3,0) = omegad;
        qddP.block<12,1>(6,0) = qdd; // these are measured joint acceleration, if not available they're set to zero

        Eigen::Matrix3d foot_jacobian = feet_jacs_.getFootJacobian(qP, leg);
        Eigen::Vector3d tau_dyn_leg = dynamics_.getRNEA(qP, qdP, qddP, leg); 
        M_leg = dynamics_.getInertiaMatrix(qP, qdP, qddP, leg); // inertia matrix
        c_leg = dynamics_.getNonLinear(qP, qdP, qddP, leg); // C(q,qd)*qd + G(q)
        
        // 3 joints of the LF leg, on a 3x1 vector;
        Eigen::Vector3d tau_leg = quadruped::getLegJointState(LegID(leg), tau);
        Eigen::Matrix3d inv_jac = (foot_jacobian.transpose()).inverse();

        foot_grf = - inv_jac * (tau_leg - tau_dyn_leg);

        #if DEBUG_MODE    
        if(!foot_grf.allFinite()){
            std::cerr << "ERROR: For leg " << leg << " grf is " << foot_grf;
        }
        else{
            std::cerr << "\n \n";
            std::cerr << "INFO: For leg " << leg << std::endl;
            // std::cerr << "xdd = " << xdd.transpose() << std::endl;
            // std::cerr << "gravity = " << gravity_base.segment(rbd::LX, 3).transpose() << std::endl;
            // std::cerr << "M_leg: " << std::endl << M_leg << std::endl;
            std::cerr << "tau_leg: " << tau_leg.transpose() << std::endl;
            std::cerr << "tau_dyn_leg: " << tau_dyn_leg.transpose() << std::endl;
            std::cerr << "c_leg: " << c_leg.transpose() << std::endl;
            std::cerr << "Jacobian: " << std::endl << foot_jacobian << std::endl;
            std::cerr << "inv_jac: " << std::endl << inv_jac << std::endl;
            std::cerr << "foot_grf: " << foot_grf.transpose() << std::endl;
        }
        #endif

        return foot_grf.allFinite();
}
}  // namespace estimator_quad
}  // namespace pronto
