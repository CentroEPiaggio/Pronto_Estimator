// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from velocity_command_msgs:msg/SimpleVelocityCommand.idl
// generated code does not contain a copyright notice

#ifndef VELOCITY_COMMAND_MSGS__MSG__DETAIL__SIMPLE_VELOCITY_COMMAND__TRAITS_HPP_
#define VELOCITY_COMMAND_MSGS__MSG__DETAIL__SIMPLE_VELOCITY_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "velocity_command_msgs/msg/detail/simple_velocity_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace velocity_command_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SimpleVelocityCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: velocity_forward
  {
    out << "velocity_forward: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_forward, out);
    out << ", ";
  }

  // member: velocity_lateral
  {
    out << "velocity_lateral: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_lateral, out);
    out << ", ";
  }

  // member: yaw_rate
  {
    out << "yaw_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_rate, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SimpleVelocityCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: velocity_forward
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity_forward: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_forward, out);
    out << "\n";
  }

  // member: velocity_lateral
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity_lateral: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_lateral, out);
    out << "\n";
  }

  // member: yaw_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_rate, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SimpleVelocityCommand & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace velocity_command_msgs

namespace rosidl_generator_traits
{

[[deprecated("use velocity_command_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const velocity_command_msgs::msg::SimpleVelocityCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  velocity_command_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use velocity_command_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const velocity_command_msgs::msg::SimpleVelocityCommand & msg)
{
  return velocity_command_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<velocity_command_msgs::msg::SimpleVelocityCommand>()
{
  return "velocity_command_msgs::msg::SimpleVelocityCommand";
}

template<>
inline const char * name<velocity_command_msgs::msg::SimpleVelocityCommand>()
{
  return "velocity_command_msgs/msg/SimpleVelocityCommand";
}

template<>
struct has_fixed_size<velocity_command_msgs::msg::SimpleVelocityCommand>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<velocity_command_msgs::msg::SimpleVelocityCommand>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<velocity_command_msgs::msg::SimpleVelocityCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VELOCITY_COMMAND_MSGS__MSG__DETAIL__SIMPLE_VELOCITY_COMMAND__TRAITS_HPP_
