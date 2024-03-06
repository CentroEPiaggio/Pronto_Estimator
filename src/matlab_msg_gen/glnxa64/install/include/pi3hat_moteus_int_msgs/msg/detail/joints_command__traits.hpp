// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pi3hat_moteus_int_msgs:msg/JointsCommand.idl
// generated code does not contain a copyright notice

#ifndef PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__JOINTS_COMMAND__TRAITS_HPP_
#define PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__JOINTS_COMMAND__TRAITS_HPP_

#include "pi3hat_moteus_int_msgs/msg/detail/joints_command__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<pi3hat_moteus_int_msgs::msg::JointsCommand>()
{
  return "pi3hat_moteus_int_msgs::msg::JointsCommand";
}

template<>
inline const char * name<pi3hat_moteus_int_msgs::msg::JointsCommand>()
{
  return "pi3hat_moteus_int_msgs/msg/JointsCommand";
}

template<>
struct has_fixed_size<pi3hat_moteus_int_msgs::msg::JointsCommand>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<pi3hat_moteus_int_msgs::msg::JointsCommand>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<pi3hat_moteus_int_msgs::msg::JointsCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__JOINTS_COMMAND__TRAITS_HPP_