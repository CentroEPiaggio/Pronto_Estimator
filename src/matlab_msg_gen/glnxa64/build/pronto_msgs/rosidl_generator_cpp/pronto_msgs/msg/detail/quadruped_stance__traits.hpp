// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pronto_msgs:msg/QuadrupedStance.idl
// generated code does not contain a copyright notice

#ifndef PRONTO_MSGS__MSG__DETAIL__QUADRUPED_STANCE__TRAITS_HPP_
#define PRONTO_MSGS__MSG__DETAIL__QUADRUPED_STANCE__TRAITS_HPP_

#include "pronto_msgs/msg/detail/quadruped_stance__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<pronto_msgs::msg::QuadrupedStance>()
{
  return "pronto_msgs::msg::QuadrupedStance";
}

template<>
inline const char * name<pronto_msgs::msg::QuadrupedStance>()
{
  return "pronto_msgs/msg/QuadrupedStance";
}

template<>
struct has_fixed_size<pronto_msgs::msg::QuadrupedStance>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<pronto_msgs::msg::QuadrupedStance>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<pronto_msgs::msg::QuadrupedStance>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PRONTO_MSGS__MSG__DETAIL__QUADRUPED_STANCE__TRAITS_HPP_
