// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rviz_legged_msgs:msg/FrictionCones.idl
// generated code does not contain a copyright notice

#ifndef RVIZ_LEGGED_MSGS__MSG__DETAIL__FRICTION_CONES__BUILDER_HPP_
#define RVIZ_LEGGED_MSGS__MSG__DETAIL__FRICTION_CONES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rviz_legged_msgs/msg/detail/friction_cones__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rviz_legged_msgs
{

namespace msg
{

namespace builder
{

class Init_FrictionCones_friction_cones
{
public:
  explicit Init_FrictionCones_friction_cones(::rviz_legged_msgs::msg::FrictionCones & msg)
  : msg_(msg)
  {}
  ::rviz_legged_msgs::msg::FrictionCones friction_cones(::rviz_legged_msgs::msg::FrictionCones::_friction_cones_type arg)
  {
    msg_.friction_cones = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rviz_legged_msgs::msg::FrictionCones msg_;
};

class Init_FrictionCones_header
{
public:
  Init_FrictionCones_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FrictionCones_friction_cones header(::rviz_legged_msgs::msg::FrictionCones::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_FrictionCones_friction_cones(msg_);
  }

private:
  ::rviz_legged_msgs::msg::FrictionCones msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rviz_legged_msgs::msg::FrictionCones>()
{
  return rviz_legged_msgs::msg::builder::Init_FrictionCones_header();
}

}  // namespace rviz_legged_msgs

#endif  // RVIZ_LEGGED_MSGS__MSG__DETAIL__FRICTION_CONES__BUILDER_HPP_
