#include "pronto_estimator_quadruped/dynamics.hpp"
#include "pronto_core/definitions.hpp"

namespace pronto{
namespace estimator_quad{

    Dynamics::Dynamics(pinocchio::Model & model, pinocchio::Data & data, std::vector<std::string> joint_names) :
            model_(model), data_(data)
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

        #if DEBUG_MODE
        std::cerr << "Joint_states: \n";
        for (int i = 0; i < joint_names.size(); i++)
        {
            std::cerr << joint_names[i] << std::endl;
        }

        std::cerr << "Joints_sorted: \n";
        for (int i = 0; i < joint_sorted.size(); i++)
        {
            std::cerr << joint_sorted[i] << std::endl;
        }

        std::cerr << "Joints_order: ";
        for (int i = 0; i < joints_order_.size(); i++)
        {
            std::cerr << joints_order_[i] << " ";
        }
        std::cerr << std::endl << std::endl;
        #endif
    }

    void Dynamics::updateDynamics(const JointStatePinocchio& q, const JointVelocityPinocchio& qd, const JointVelocityPinocchio& qdd)
    {           
        JointStatePinocchio q2 = q;
        JointVelocityPinocchio qd2 = qd;
        JointVelocityPinocchio qdd2 = qdd;
        for(int i = 0; i < joints_order_.size(); i++)
        {
            q2[7+i] = q[joints_order_[i]+7];
            qd2[6+i] = qd[joints_order_[i]+6];
            qdd2[6+i] = qdd[joints_order_[i]+6];
        }
        pinocchio::rnea(model_, data_, q2, qd2, qdd2);

        #if DEBUG_MODE  
            pinocchio::crba(model_, data_, q);
            // crba() computes only the upper triangular part of the matrix M, to obtain the full matrix we apply:
            data_.M.triangularView<Eigen::StrictlyLower>() = data_.M.transpose().triangularView<Eigen::StrictlyLower>();
            pinocchio::nonLinearEffects(model_, data_, q, qd);

            std::cerr << "DYNAMICS: Joints state check: \n";
            std::cerr << "Joints state: " << q.transpose() << std::endl;
            std::cerr << "Joints reoerdered: " << q2.transpose() << std::endl;
        #endif

        prev_q = q;

        // Reordering joints according to joints_order_
        tau_.block<6,1>(0,0) = data_.tau.block<6,1>(0,0);
        for(int i = 0; i < joints_order_.size(); i++)
        {
            tau_[joints_order_[i]+6] = data_.tau[i+6];
        }
    }

    Eigen::Matrix3d Dynamics::getInertiaMatrix(const JointStatePinocchio& q,
                                                const JointVelocityPinocchio& qd,
                                                const JointVelocityPinocchio& qdd,
                                                const LegID& leg)
    {
        // This function doesn't work with joints in non alphabetical order
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
        // This function doesn't work with joints in non alphabetical order
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

        return tau_.block<3,1>(6 + leg * 3, 0);
    }


} // namespace estimator_quad
} // namespace pronto