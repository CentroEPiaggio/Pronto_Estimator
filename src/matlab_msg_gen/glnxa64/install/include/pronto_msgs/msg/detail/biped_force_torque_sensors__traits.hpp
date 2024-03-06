// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pronto_msgs:msg/BipedForceTorqueSensors.idl
// generated code does not contain a copyright notice

#ifndef PRONTO_MSGS__MSG__DETAIL__BIPED_FORCE_TORQUE_SENSORS__TRAITS_HPP_
#define PRONTO_MSGS__MSG__DETAIL__BIPED_FORCE_TORQUE_SENSORS__TRAITS_HPP_

#include "pronto_msgs/msg/detail/biped_force_torque_sensors__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'l_foot'
// Member 'r_foot'
// Member 'l_hand'
// Member 'r_hand'
#include "geometry_msgs/msg/detail/wrench__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<pronto_msgs::msg::BipedForceTorqueSensors>()
{
  return "pronto_msgs::msg::BipedForceTorqueSensors";
}

template<>
inline const char * name<pronto_msgs::msg::BipedForceTorqueSensors>()
{
  return "pronto_msgs/msg/BipedForceTorqueSensors";
}

template<>
struct has_fixed_size<pronto_msgs::msg::BipedForceTorqueSensors>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Wrench>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<pronto_msgs::msg::BipedForceTorqueSensors>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Wrench>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<pronto_msgs::msg::BipedForceTorqueSensors>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PRONTO_MSGS__MSG__DETAIL__BIPED_FORCE_TORQUE_SENSORS__TRAITS_HPP_
