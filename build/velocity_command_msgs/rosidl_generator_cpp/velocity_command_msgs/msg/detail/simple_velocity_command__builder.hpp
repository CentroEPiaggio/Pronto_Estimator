// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from velocity_command_msgs:msg/SimpleVelocityCommand.idl
// generated code does not contain a copyright notice

#ifndef VELOCITY_COMMAND_MSGS__MSG__DETAIL__SIMPLE_VELOCITY_COMMAND__BUILDER_HPP_
#define VELOCITY_COMMAND_MSGS__MSG__DETAIL__SIMPLE_VELOCITY_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "velocity_command_msgs/msg/detail/simple_velocity_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace velocity_command_msgs
{

namespace msg
{

namespace builder
{

class Init_SimpleVelocityCommand_yaw_rate
{
public:
  explicit Init_SimpleVelocityCommand_yaw_rate(::velocity_command_msgs::msg::SimpleVelocityCommand & msg)
  : msg_(msg)
  {}
  ::velocity_command_msgs::msg::SimpleVelocityCommand yaw_rate(::velocity_command_msgs::msg::SimpleVelocityCommand::_yaw_rate_type arg)
  {
    msg_.yaw_rate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::velocity_command_msgs::msg::SimpleVelocityCommand msg_;
};

class Init_SimpleVelocityCommand_velocity_lateral
{
public:
  explicit Init_SimpleVelocityCommand_velocity_lateral(::velocity_command_msgs::msg::SimpleVelocityCommand & msg)
  : msg_(msg)
  {}
  Init_SimpleVelocityCommand_yaw_rate velocity_lateral(::velocity_command_msgs::msg::SimpleVelocityCommand::_velocity_lateral_type arg)
  {
    msg_.velocity_lateral = std::move(arg);
    return Init_SimpleVelocityCommand_yaw_rate(msg_);
  }

private:
  ::velocity_command_msgs::msg::SimpleVelocityCommand msg_;
};

class Init_SimpleVelocityCommand_velocity_forward
{
public:
  Init_SimpleVelocityCommand_velocity_forward()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SimpleVelocityCommand_velocity_lateral velocity_forward(::velocity_command_msgs::msg::SimpleVelocityCommand::_velocity_forward_type arg)
  {
    msg_.velocity_forward = std::move(arg);
    return Init_SimpleVelocityCommand_velocity_lateral(msg_);
  }

private:
  ::velocity_command_msgs::msg::SimpleVelocityCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::velocity_command_msgs::msg::SimpleVelocityCommand>()
{
  return velocity_command_msgs::msg::builder::Init_SimpleVelocityCommand_velocity_forward();
}

}  // namespace velocity_command_msgs

#endif  // VELOCITY_COMMAND_MSGS__MSG__DETAIL__SIMPLE_VELOCITY_COMMAND__BUILDER_HPP_
