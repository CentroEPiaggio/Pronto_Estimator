#include "pronto_quadruped_ros/conversions.hpp"

namespace pronto {
namespace quadruped {

bool jointStateFromROS(const sensor_msgs::msg::JointState& msg,
                       uint64_t& utime,
                       JointState& q,
                       JointState& qd,
                       JointState& qdd,
                       JointState& tau,
                       rclcpp::Node::SharedPtr nh_)
{
    // if the size of the joint state message does not match our own,
    // we silently return an invalid update
    std::size_t size = static_cast<size_t>(q.size());

    if (msg.position.size() != size ||
        msg.velocity.size() != size ||
        msg.effort.size() != size){
        RCLCPP_WARN(nh_->get_logger(), "Joint State is expected %zu joints but %zu / %zu / %zu are provided.",
                         size, msg.position.size(), msg.velocity.size(), msg.effort.size());
        return false;
    }

    std::vector<std::string> joint_names;
    if(!nh_->get_parameter("joint_names", joint_names)){
      RCLCPP_ERROR(nh_->get_logger(), "JOINT STATES CONVERSION: Couldn't get joint names, conversion not possible.");
      return false;
    }

    // store message time in microseconds
    utime = msg.header.stamp.nanosec / 1000;
    for(int i=0; i<12; i++){

      // joint states are not in order, we need to sort them
      auto j = std::find(msg.name.begin(), msg.name.end(), joint_names[i]);

      if(j!=msg.name.end()){
        int index = j - msg.name.begin();
        q(i) = msg.position[index];
        qd(i) = msg.velocity[index];
        tau(i) = msg.effort[index];
      }
      else{
        RCLCPP_ERROR(nh_->get_logger(), "JOINT STATES CONVERSION: Joint named %s doesn't exists.", joint_names[i]);
        return false;
      }
      
    }
    qdd.setZero(); // TODO compute the acceleration

    return true;
}

bool jointStateWithAccelerationFromROS(const pronto_msgs::msg::JointStateWithAcceleration& msg,
                               uint64_t& utime,
                               JointState& q,
                               JointState& qd,
                               JointState& qdd,
                               JointState& tau,
                               rclcpp::Node::SharedPtr nh_)
{
    // if the size of the joint state message does not match our own,
    // we silently return an invalid update
    std::size_t size = static_cast<size_t>(q.size());

    if (msg.position.size() != size ||
        msg.velocity.size() != size ||
        msg.acceleration.size() != size ||
        msg.effort.size() != size){
        RCLCPP_WARN(rclcpp::get_logger("jointStateWithAccelerationFromROS"), "Joint State is expected %zu joints but %zu / %zu / %zu / %zu are provided.",
                         size, msg.position.size(), msg.velocity.size(), msg.acceleration.size(), msg.effort.size());
        return false;
    }
    
    std::vector<std::string> joint_names;
    if(!nh_->get_parameter("joint_names", joint_names)){
      RCLCPP_ERROR(nh_->get_logger(), "JOINT STATES CONVERSION: Couldn't get joint names, conversion not possible.");
      return false;
    }

    // store message time in microseconds
    utime = msg.header.stamp.nanosec / 1000;
    for(int i=0; i<12; i++){

      // joint states are not in order, we need to sort them
      auto j = std::find(msg.name.begin(), msg.name.end(), joint_names[i]);

      if(j!=msg.name.end()){
        int index = j - msg.name.begin();
        q(i) = msg.position[index];
        qd(i) = msg.velocity[index];
        qdd(i) = msg.acceleration[index];
        tau(i) = msg.effort[index];
      }
      else{
        RCLCPP_ERROR(nh_->get_logger(), "JOINT STATES CONVERSION: Joint named %s doesn't exists.", joint_names[i]);
        return false;
      }
    }

    return true;
}

}  // namespace quadruped
}  // namespace pronto
