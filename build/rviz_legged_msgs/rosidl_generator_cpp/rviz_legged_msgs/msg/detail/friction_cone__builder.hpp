// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rviz_legged_msgs:msg/FrictionCone.idl
// generated code does not contain a copyright notice

#ifndef RVIZ_LEGGED_MSGS__MSG__DETAIL__FRICTION_CONE__BUILDER_HPP_
#define RVIZ_LEGGED_MSGS__MSG__DETAIL__FRICTION_CONE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rviz_legged_msgs/msg/detail/friction_cone__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rviz_legged_msgs
{

namespace msg
{

namespace builder
{

class Init_FrictionCone_friction_coefficient
{
public:
  explicit Init_FrictionCone_friction_coefficient(::rviz_legged_msgs::msg::FrictionCone & msg)
  : msg_(msg)
  {}
  ::rviz_legged_msgs::msg::FrictionCone friction_coefficient(::rviz_legged_msgs::msg::FrictionCone::_friction_coefficient_type arg)
  {
    msg_.friction_coefficient = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rviz_legged_msgs::msg::FrictionCone msg_;
};

class Init_FrictionCone_normal_direction
{
public:
  explicit Init_FrictionCone_normal_direction(::rviz_legged_msgs::msg::FrictionCone & msg)
  : msg_(msg)
  {}
  Init_FrictionCone_friction_coefficient normal_direction(::rviz_legged_msgs::msg::FrictionCone::_normal_direction_type arg)
  {
    msg_.normal_direction = std::move(arg);
    return Init_FrictionCone_friction_coefficient(msg_);
  }

private:
  ::rviz_legged_msgs::msg::FrictionCone msg_;
};

class Init_FrictionCone_header
{
public:
  Init_FrictionCone_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FrictionCone_normal_direction header(::rviz_legged_msgs::msg::FrictionCone::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_FrictionCone_normal_direction(msg_);
  }

private:
  ::rviz_legged_msgs::msg::FrictionCone msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rviz_legged_msgs::msg::FrictionCone>()
{
  return rviz_legged_msgs::msg::builder::Init_FrictionCone_header();
}

}  // namespace rviz_legged_msgs

#endif  // RVIZ_LEGGED_MSGS__MSG__DETAIL__FRICTION_CONE__BUILDER_HPP_
