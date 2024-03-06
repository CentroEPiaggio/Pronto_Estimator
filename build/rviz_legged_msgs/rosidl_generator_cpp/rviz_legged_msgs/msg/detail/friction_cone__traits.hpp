// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rviz_legged_msgs:msg/FrictionCone.idl
// generated code does not contain a copyright notice

#ifndef RVIZ_LEGGED_MSGS__MSG__DETAIL__FRICTION_CONE__TRAITS_HPP_
#define RVIZ_LEGGED_MSGS__MSG__DETAIL__FRICTION_CONE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rviz_legged_msgs/msg/detail/friction_cone__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'normal_direction'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace rviz_legged_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const FrictionCone & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: normal_direction
  {
    out << "normal_direction: ";
    to_flow_style_yaml(msg.normal_direction, out);
    out << ", ";
  }

  // member: friction_coefficient
  {
    out << "friction_coefficient: ";
    rosidl_generator_traits::value_to_yaml(msg.friction_coefficient, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FrictionCone & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: normal_direction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "normal_direction:\n";
    to_block_style_yaml(msg.normal_direction, out, indentation + 2);
  }

  // member: friction_coefficient
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "friction_coefficient: ";
    rosidl_generator_traits::value_to_yaml(msg.friction_coefficient, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FrictionCone & msg, bool use_flow_style = false)
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

}  // namespace rviz_legged_msgs

namespace rosidl_generator_traits
{

[[deprecated("use rviz_legged_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rviz_legged_msgs::msg::FrictionCone & msg,
  std::ostream & out, size_t indentation = 0)
{
  rviz_legged_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rviz_legged_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const rviz_legged_msgs::msg::FrictionCone & msg)
{
  return rviz_legged_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rviz_legged_msgs::msg::FrictionCone>()
{
  return "rviz_legged_msgs::msg::FrictionCone";
}

template<>
inline const char * name<rviz_legged_msgs::msg::FrictionCone>()
{
  return "rviz_legged_msgs/msg/FrictionCone";
}

template<>
struct has_fixed_size<rviz_legged_msgs::msg::FrictionCone>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<rviz_legged_msgs::msg::FrictionCone>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<rviz_legged_msgs::msg::FrictionCone>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RVIZ_LEGGED_MSGS__MSG__DETAIL__FRICTION_CONE__TRAITS_HPP_
