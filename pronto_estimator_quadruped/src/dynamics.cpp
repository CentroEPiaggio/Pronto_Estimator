#include "pronto_estimator_quadruped/dynamics.hpp"

namespace pronto{
namespace estimator_quad{

    Dynamics::Dynamics(pinocchio::Model & model, pinocchio::Data & data) :
            model_(model), data_(data)
    {
        prev_q.setZero();
    }

    void Dynamics::updateDynamics(const JointStatePinocchio& q, const JointVelocityPinocchio& qd, const JointVelocityPinocchio& qdd)
    {
        pinocchio::crba(model_, data_, q);        
        // crba() computes only the upper triangular part of the matrix M, to obtain the full matrix we apply:
        data_.M.triangularView<Eigen::StrictlyLower>() = data_.M.transpose().triangularView<Eigen::StrictlyLower>();

        pinocchio::nonLinearEffects(model_, data_, q, qd); // this is only for debug

        pinocchio::rnea(model_, data_, q, qd, qdd);

        prev_q = q;
    }

    // void Dynamics::computeInertiaMatrix(const JointStatePinocchio& q)
    // {
    //     pinocchio::crba(model_, data_, q);
        
    //     // crba() computes only the upper triangular part of the matrix M, to obtain the full matrix we apply:
    //     data_.M.triangularView<Eigen::StrictlyLower>() = data_.M.transpose().triangularView<Eigen::StrictlyLower>();
        

    //     prev_q = q;
    // }

    // Eigen::Matrix3d Dynamics::getInertiaMatrix(const JointStatePinocchio& q, const LegID& leg)
    // {
    //     if(q != prev_q){
    //         Dynamics::computeInertiaMatrix(q);
    //     }
       
    //     return data_.M.block<3, 3>(leg * 3, leg * 3);
    // }

    // void Dynamics::computeDynamics(const JointStatePinocchio& q, const JointVelocityPinocchio& qd){

    //     // TODO: check if it's possible to use rnea() for a faster calculation
    //     pinocchio::nonLinearEffects(model_, data_, q, qd);
        
    //     prev_q = q;  
    // }

    // Dynamics::Vector3d Dynamics::getNonLinear(const JointStatePinocchio& q, const JointVelocityPinocchio& qd, const LegID& leg)
    // {
    //     // if(q != prev_q) // suppose that if q didn't change, qd didn't change as well
    //     // { 
    //     //     Dynamics::computeDynamics(q, qd);
    //     // }
    //     Dynamics::computeDynamics(q, qd);
    //     return data_.nle.block<3, 1>(6 + leg * 3, 0);
    // }

    Eigen::Matrix3d Dynamics::getInertiaMatrix(const JointStatePinocchio& q,
                                                const JointVelocityPinocchio& qd,
                                                const JointVelocityPinocchio& qdd,
                                                const LegID& leg)
    {
        if(q != prev_q)
        {
            updateDynamics(q, qd, qdd);
        }
        return data_.M.block<3,3>(6 + leg * 3, 6 + leg * 3);
    }

    Dynamics::Vector3d Dynamics::getNonLinear(const JointStatePinocchio& q,
                                                const JointVelocityPinocchio& qd,
                                                const JointVelocityPinocchio& qdd,
                                                const LegID& leg)
    {
        if(q != prev_q)
        {
            updateDynamics(q, qd, qdd);
        }
        return data_.nle.block<3, 1>(6 + leg * 3, 0);
    }

    Dynamics::Vector3d Dynamics::getRNEA(const JointStatePinocchio& q,
                                        const JointVelocityPinocchio& qd,
                                        const JointVelocityPinocchio& qdd,
                                        const LegID& leg)
    {
        if(q != prev_q)
        {
            updateDynamics(q, qd, qdd);
        }

        return data_.tau.block<3,1>(6 + leg * 3, 0);
    }


} // namespace estimator_quad
} // namespace pronto