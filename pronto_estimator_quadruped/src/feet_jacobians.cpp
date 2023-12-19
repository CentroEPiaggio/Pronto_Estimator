#include "pronto_estimator_quadruped/feet_jacobians.hpp"

using namespace iit::rbd;

namespace pronto{
namespace estimator_quad{

    typedef pronto::quadruped::Matrix3d Matrix3d;

    FeetJacobians::FeetJacobians(pinocchio::Model & model, pinocchio::Data & data, std::vector<std::string> feet_names) :
            model_(model), data_(data), feet_names_(feet_names)
    {
        prev_q.setZero();
    }

    void FeetJacobians::updateConfiguration(const JointStatePinocchio& q)
    {            
        // compute legs update if joint states are different from previous configuration
        // q must always be a vector of 19 elements
        pinocchio::computeJointJacobians(model_, data_, q);
        pinocchio::updateFramePlacements(model_, data_);

        // update previous configuration
        prev_q = q;
    }

    pinocchio::Data::Matrix6x FeetJacobians::ComputeJacobian(const JointStatePinocchio & q,
                                                            const LegID& leg)
    {
        if(q != prev_q) // TODO: check why the function pinocchio::isSameConfiguration does not work
        {
            updateConfiguration(q);
        }

        leg_id = model_.getFrameId(feet_names_[leg]);

        if(leg_id < model_.frames.size()){                
        pinocchio::getFrameJacobian(model_, data_, leg_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);
        return J;
        }
        else{
            std::cerr << "[FeetJacobians::ComputeJacobian(...)] "
            << "ERROR: foot not found. Returning zero."
            << std::endl;

            J.setZero();
            return J;
        }
    }

    FeetJacobians::FootJac FeetJacobians::getFootJacobian(const JointStatePinocchio& q, const LegID& leg)
    {
        J = ComputeJacobian(q, leg);

        // std::cerr << "Leg: " << leg << std::endl;
        // std::cerr << "Jacobiano Completo: " << std::endl;
        // std::cerr << J << std::endl;
        
        return J.block<3,3>(0, 6 + leg*3);
    }

    FeetJacobians::FootJac FeetJacobians::getFootJacobianAngular(const JointStatePinocchio& q, const LegID& leg)
    {
        J = ComputeJacobian(q, leg);        
        return J.block<3,3>(3, 6 + leg*3);
    }


    FeetJacobians::FootJac FeetJacobians::getFootJacobian(const JointState& q, const LegID& leg)
    {
        // checking if this function is used
        std::cerr << "ERROR CHECK: Program entered wrong \"getFootJacobian\" function." << std::endl;
    }

    FeetJacobians::FootJac FeetJacobians::getFootJacobianAngular(const JointState& q, const LegID& leg)
    {
        // checking if this function is used
        std::cerr << "ERROR CHECK: Program entered wrong \"getFootJacobianAngular\" function." << std::endl;
    }


    Vector3d FeetJacobians::getFootPos( const JointStatePinocchio& q, 
                                        const LegID& leg)
    {
        if(q != prev_q)
        {
            updateConfiguration(q);
        }

        leg_id = model_.getFrameId(feet_names_[leg]);

        if(leg_id < model_.frames.size()){ 
            T = data_.oMf[leg_id];
            return T.translation();    
        }
        else{
            std::cerr << "[FeetJacobians::getFootPos(...)] "
            << "ERROR: foot not found. Returning zero."
            << std::endl;

            T.setIdentity();
            return T.translation();
        }            
    }

    Matrix3d FeetJacobians::getFootOrientation( const JointStatePinocchio& q, 
                                        const LegID& leg)
    {
        if(q != prev_q)
        {
            updateConfiguration(q);
        }

        leg_id = model_.getFrameId(feet_names_[leg]);

        if(leg_id < model_.frames.size()){ 
            T = data_.oMf[leg_id];
            return T.rotation(); // TODO: check how rotation is defined
        }
        else{
            std::cerr << "[FeetJacobians::getFootPos(...)] "
            << "ERROR: foot not found. Returning zero."
            << std::endl;

            T.setIdentity();
            return T.rotation();
        }   
    }
} // namespace estimator_quad
} // namespace pronto