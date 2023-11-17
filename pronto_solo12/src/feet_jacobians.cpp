#include "pronto_solo12/feet_jacobians.hpp"

using namespace iit::rbd;

namespace pronto{
namespace solo{

    typedef pronto::quadruped::Matrix3d Matrix3d;

    void FeetJacobians::updateConfiguration(const JointState& q)
    {            
        // compute legs update if joint states are different from previous configuration
        // q must always be a vector of 12 elements
        pinocchio::computeJointJacobians(model_, data_, q);
        pinocchio::updateFramePlacements(model_, data_);

        // update previous configuration
        prev_q = q;
    }

    pinocchio::Data::Matrix6x FeetJacobians::ComputeJacobian(const JointState & q,
                                                            const LegID& leg)
        {
            if(q != prev_q) // TODO: check why the function pinocchio::isSameConfiguration does not work
            {
                updateConfiguration(q);
            }

            switch (leg)
            {
                case pronto::quadruped::LF:
                    leg_id = model_.getFrameId("LF_FOOT");
                    pinocchio::getFrameJacobian(model_, data_, leg_id, pinocchio::ReferenceFrame::WORLD, J);
                    break;
                    
                case pronto::quadruped::RF:
                    leg_id = model_.getFrameId("RF_FOOT");
                    pinocchio::getFrameJacobian(model_, data_, leg_id, pinocchio::ReferenceFrame::WORLD, J);
                    // break;
                    
                case pronto::quadruped::LH:
                    leg_id = model_.getFrameId("LH_FOOT");
                    pinocchio::getFrameJacobian(model_, data_, leg_id, pinocchio::ReferenceFrame::WORLD, J);
                    break;
                    
                case pronto::quadruped::RH:
                    leg_id = model_.getFrameId("RH_FOOT");
                    pinocchio::getFrameJacobian(model_, data_, leg_id, pinocchio::ReferenceFrame::WORLD, J);
                    break;

                default:
                    std::cerr << "[FeetJacobians::ComputeJacobian(...)] "
                        << "ERROR: legID not recognized. Returning zero."
                        << std::endl;   
                    J.setZero(6, 12);
                    break;
                    
            }

            return J;

        }

    FeetJacobians::FootJac FeetJacobians::getFootJacobian(const JointState& q, 
                                                            const LegID& leg)
    {
        J = ComputeJacobian(q, leg);

        
        return J.block<3,3>(0,leg*3);
    }

    FeetJacobians::FootJac FeetJacobians::getFootJacobianAngular(const JointState& q, 
                                                            const LegID& leg)
    {
        J = ComputeJacobian(q, leg);

        
        return J.block<3,3>(3, leg*3);
    }

    Vector3d FeetJacobians::getFootPos( const JointState& q, 
                                        const LegID& leg)
    {
        if(q != prev_q)
        {
            updateConfiguration(q);
        }

        switch (leg)
        {
            case pronto::quadruped::LF:
                leg_id = model_.getFrameId("LF_FOOT");
                T = data_.oMf[leg_id];
                break;
                
            case pronto::quadruped::RF:
                leg_id = model_.getFrameId("RF_FOOT");
                T = data_.oMf[leg_id];
                break;
                
            case pronto::quadruped::LH:
                leg_id = model_.getFrameId("LH_FOOT");
                T = data_.oMf[leg_id];
                break;
                
            case pronto::quadruped::RH:
                leg_id = model_.getFrameId("RH_FOOT");
                T = data_.oMf[leg_id];
                break;

            default:
               std::cerr << "[ForwardKinematics::getFootPos(...)] "
                  << "ERROR: legID not recognized. Returning identity."
                  << std::endl;       
                T.setIdentity();
                break;
        }

        return T.translation();        
    }

    Matrix3d FeetJacobians::getFootOrientation( const JointState& q, 
                                        const LegID& leg)
    {
        if(q != prev_q)
        {
            updateConfiguration(q);
        }

        switch (leg)
        {
            case pronto::quadruped::LF:
                leg_id = model_.getFrameId("LF_FOOT");
                T = data_.oMf[leg_id];
                break;
                
            case pronto::quadruped::RF:
                leg_id = model_.getFrameId("RF_FOOT");
                T = data_.oMf[leg_id];
                break;
                
            case pronto::quadruped::LH:
                leg_id = model_.getFrameId("LH_FOOT");
                T = data_.oMf[leg_id];
                break;
                
            case pronto::quadruped::RH:
                leg_id = model_.getFrameId("RH_FOOT");
                T = data_.oMf[leg_id];
                break;

            default:
                std::cerr << "[ForwardKinematics::getFootOrientation(...)] "
                  << "ERROR: legID not recognized. Returning identity."
                  << std::endl; 
                T.setIdentity();   
                break;    
        }

        return T.rotation();  // TODO: check how rotation is defined      
    }
}
}