// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from generalized_pose_msgs:msg/GeneralizedPose.idl
// generated code does not contain a copyright notice

#ifndef GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSE__BUILDER_HPP_
#define GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "generalized_pose_msgs/msg/detail/generalized_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace generalized_pose_msgs
{

namespace msg
{

namespace builder
{

class Init_GeneralizedPose_contact_feet
{
public:
  explicit Init_GeneralizedPose_contact_feet(::generalized_pose_msgs::msg::GeneralizedPose & msg)
  : msg_(msg)
  {}
  ::generalized_pose_msgs::msg::GeneralizedPose contact_feet(::generalized_pose_msgs::msg::GeneralizedPose::_contact_feet_type arg)
  {
    msg_.contact_feet = std::move(arg);
    return std::move(msg_);
  }

private:
  ::generalized_pose_msgs::msg::GeneralizedPose msg_;
};

class Init_GeneralizedPose_feet_pos
{
public:
  explicit Init_GeneralizedPose_feet_pos(::generalized_pose_msgs::msg::GeneralizedPose & msg)
  : msg_(msg)
  {}
  Init_GeneralizedPose_contact_feet feet_pos(::generalized_pose_msgs::msg::GeneralizedPose::_feet_pos_type arg)
  {
    msg_.feet_pos = std::move(arg);
    return Init_GeneralizedPose_contact_feet(msg_);
  }

private:
  ::generalized_pose_msgs::msg::GeneralizedPose msg_;
};

class Init_GeneralizedPose_feet_vel
{
public:
  explicit Init_GeneralizedPose_feet_vel(::generalized_pose_msgs::msg::GeneralizedPose & msg)
  : msg_(msg)
  {}
  Init_GeneralizedPose_feet_pos feet_vel(::generalized_pose_msgs::msg::GeneralizedPose::_feet_vel_type arg)
  {
    msg_.feet_vel = std::move(arg);
    return Init_GeneralizedPose_feet_pos(msg_);
  }

private:
  ::generalized_pose_msgs::msg::GeneralizedPose msg_;
};

class Init_GeneralizedPose_feet_acc
{
public:
  explicit Init_GeneralizedPose_feet_acc(::generalized_pose_msgs::msg::GeneralizedPose & msg)
  : msg_(msg)
  {}
  Init_GeneralizedPose_feet_vel feet_acc(::generalized_pose_msgs::msg::GeneralizedPose::_feet_acc_type arg)
  {
    msg_.feet_acc = std::move(arg);
    return Init_GeneralizedPose_feet_vel(msg_);
  }

private:
  ::generalized_pose_msgs::msg::GeneralizedPose msg_;
};

class Init_GeneralizedPose_base_quat
{
public:
  explicit Init_GeneralizedPose_base_quat(::generalized_pose_msgs::msg::GeneralizedPose & msg)
  : msg_(msg)
  {}
  Init_GeneralizedPose_feet_acc base_quat(::generalized_pose_msgs::msg::GeneralizedPose::_base_quat_type arg)
  {
    msg_.base_quat = std::move(arg);
    return Init_GeneralizedPose_feet_acc(msg_);
  }

private:
  ::generalized_pose_msgs::msg::GeneralizedPose msg_;
};

class Init_GeneralizedPose_base_angvel
{
public:
  explicit Init_GeneralizedPose_base_angvel(::generalized_pose_msgs::msg::GeneralizedPose & msg)
  : msg_(msg)
  {}
  Init_GeneralizedPose_base_quat base_angvel(::generalized_pose_msgs::msg::GeneralizedPose::_base_angvel_type arg)
  {
    msg_.base_angvel = std::move(arg);
    return Init_GeneralizedPose_base_quat(msg_);
  }

private:
  ::generalized_pose_msgs::msg::GeneralizedPose msg_;
};

class Init_GeneralizedPose_base_pos
{
public:
  explicit Init_GeneralizedPose_base_pos(::generalized_pose_msgs::msg::GeneralizedPose & msg)
  : msg_(msg)
  {}
  Init_GeneralizedPose_base_angvel base_pos(::generalized_pose_msgs::msg::GeneralizedPose::_base_pos_type arg)
  {
    msg_.base_pos = std::move(arg);
    return Init_GeneralizedPose_base_angvel(msg_);
  }

private:
  ::generalized_pose_msgs::msg::GeneralizedPose msg_;
};

class Init_GeneralizedPose_base_vel
{
public:
  explicit Init_GeneralizedPose_base_vel(::generalized_pose_msgs::msg::GeneralizedPose & msg)
  : msg_(msg)
  {}
  Init_GeneralizedPose_base_pos base_vel(::generalized_pose_msgs::msg::GeneralizedPose::_base_vel_type arg)
  {
    msg_.base_vel = std::move(arg);
    return Init_GeneralizedPose_base_pos(msg_);
  }

private:
  ::generalized_pose_msgs::msg::GeneralizedPose msg_;
};

class Init_GeneralizedPose_base_acc
{
public:
  Init_GeneralizedPose_base_acc()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GeneralizedPose_base_vel base_acc(::generalized_pose_msgs::msg::GeneralizedPose::_base_acc_type arg)
  {
    msg_.base_acc = std::move(arg);
    return Init_GeneralizedPose_base_vel(msg_);
  }

private:
  ::generalized_pose_msgs::msg::GeneralizedPose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::generalized_pose_msgs::msg::GeneralizedPose>()
{
  return generalized_pose_msgs::msg::builder::Init_GeneralizedPose_base_acc();
}

}  // namespace generalized_pose_msgs

#endif  // GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSE__BUILDER_HPP_
