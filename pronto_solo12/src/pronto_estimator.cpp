#include <rclcpp/rclcpp.hpp> 

// #include "pronto_ros/pronto_node.hpp"
#include "pronto_solo12/pronto_node.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "pronto_solo12/feet_contact_forces.hpp"
#include "pronto_solo12/feet_jacobians.hpp"
#include "pronto_solo12/forward_kinematics.hpp"
#include "pronto_solo12/dynamics.hpp"


using namespace pronto;

const std::string urdf_file = "/home/simone/solo12/src/solo_robot_description/urdf/again/solo12.urdf";
pinocchio::Model robot_model;
const pinocchio::JointModelFreeFlyer root_joint;

int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);

    pinocchio::urdf::buildModel(urdf_file, root_joint, robot_model);
    // pinocchio::urdf::buildModel(urdf_file, robot_model);
    pinocchio::Data data(robot_model); 
    robot_model.gravity.linear() << 0.0, 0.0, -9.81;

    // for(int i = 0; i < robot_model.names.size(); i++){
    //     std::cerr << "Model names[" << i << "] = " << robot_model.names[i] << std::endl;
    // }
    
    // std::cerr << "Floating base name: " << robot_model.names[1];
    // std::cerr << robot_model.joints[1];

    solo::FeetJacobians feet_jacs(robot_model, data);
    solo::ForwardKinematics fwd_kin(feet_jacs);
    solo::Dynamics dynamics(robot_model, data);
    solo::FeetContactForces feet_forces(feet_jacs, dynamics);

    // solo::ProntoNode<sensor_msgs::msg::JointState> node;

    auto node = std::make_shared<solo::ProntoNode<sensor_msgs::msg::JointState>>();

    node->run(fwd_kin, feet_jacs, feet_forces);

    return 0;
}
  
