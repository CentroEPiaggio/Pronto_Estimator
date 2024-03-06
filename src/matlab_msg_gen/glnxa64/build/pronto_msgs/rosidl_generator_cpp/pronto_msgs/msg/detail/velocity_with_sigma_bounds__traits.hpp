// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pronto_msgs:msg/VelocityWithSigmaBounds.idl
// generated code does not contain a copyright notice

#ifndef PRONTO_MSGS__MSG__DETAIL__VELOCITY_WITH_SIGMA_BOUNDS__TRAITS_HPP_
#define PRONTO_MSGS__MSG__DETAIL__VELOCITY_WITH_SIGMA_BOUNDS__TRAITS_HPP_

#include "pronto_msgs/msg/detail/velocity_with_sigma_bounds__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'velocity_plus_one_sigma'
// Member 'velocity_minus_one_sigma'
// Member 'plus_one_sigma'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<pronto_msgs::msg::VelocityWithSigmaBounds>()
{
  return "pronto_msgs::msg::VelocityWithSigmaBounds";
}

template<>
inline const char * name<pronto_msgs::msg::VelocityWithSigmaBounds>()
{
  return "pronto_msgs/msg/VelocityWithSigmaBounds";
}

template<>
struct has_fixed_size<pronto_msgs::msg::VelocityWithSigmaBounds>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<pronto_msgs::msg::VelocityWithSigmaBounds>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<pronto_msgs::msg::VelocityWithSigmaBounds>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PRONTO_MSGS__MSG__DETAIL__VELOCITY_WITH_SIGMA_BOUNDS__TRAITS_HPP_
