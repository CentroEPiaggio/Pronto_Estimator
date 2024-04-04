#include "pronto_estimator_quadruped/feet_jacobians.hpp"
#include "pronto_core/definitions.hpp"

using namespace iit::rbd;

namespace pronto{
namespace estimator_quad{

    typedef pronto::quadruped::Matrix3d Matrix3d;

    FeetJacobians::FeetJacobians(pinocchio::Model & model, pinocchio::Data & data, std::vector<std::string> feet_names, std::vector<std::string> joint_names) :
            model_(model), data_(data), feet_names_(feet_names)
    {
        prev_q.setZero();

        // Pinocchio needs joints sorted in alphabetic order
        std::vector<std::string> joint_sorted(joint_names);
        std::sort(joint_sorted.begin(), joint_sorted.end());

        for(int i = 0; i < joint_names.size(); i++)
        {
            auto index = std::find(joint_names.begin(), joint_names.end(), joint_sorted[i]);
            int j = index - joint_names.begin();
            joints_order_[i] = j; // the i-th sorted joint corresponds to the j-th joint in user order
        }
    }

    void FeetJacobians::updateConfiguration(const JointStatePinocchio& q)
    {            
        #if DEBUG_MODE

        std::cerr << "Joint states = " << q.transpose() << std::endl;
        
        #endif
        
        // compute legs update if joint states are different from previous configuration
        // q must always be a vector of 19 elements
        JointStatePinocchio q2 = q;
        q2.segment(3, 4) << 0.0, 0.0, 0.0, 1.0;
        for(int i = 0; i < joints_order_.size(); i++)
        {
            q2[7+i] = q[joints_order_[i]+7];
        }
        pinocchio::computeJointJacobians(model_, data_, q2);
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
        for(int i = 0; i < joints_order_.size(); i++)
        {
            J_ordered.block<6,1>(0, 6 + joints_order_[i]) = J.block<6,1>(0, 6 + i);
        }
        return J_ordered;
        }
        else{
            std::cerr << "[FeetJacobians::ComputeJacobian(...)] "
            << "ERROR: foot" << feet_names_[leg] << "not found. Returning zero."
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