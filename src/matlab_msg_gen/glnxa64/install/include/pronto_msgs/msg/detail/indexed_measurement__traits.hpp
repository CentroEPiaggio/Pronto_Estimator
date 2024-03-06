// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pronto_msgs:msg/IndexedMeasurement.idl
// generated code does not contain a copyright notice

#ifndef PRONTO_MSGS__MSG__DETAIL__INDEXED_MEASUREMENT__TRAITS_HPP_
#define PRONTO_MSGS__MSG__DETAIL__INDEXED_MEASUREMENT__TRAITS_HPP_

#include "pronto_msgs/msg/detail/indexed_measurement__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<pronto_msgs::msg::IndexedMeasurement>()
{
  return "pronto_msgs::msg::IndexedMeasurement";
}

template<>
inline const char * name<pronto_msgs::msg::IndexedMeasurement>()
{
  return "pronto_msgs/msg/IndexedMeasurement";
}

template<>
struct has_fixed_size<pronto_msgs::msg::IndexedMeasurement>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<pronto_msgs::msg::IndexedMeasurement>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<pronto_msgs::msg::IndexedMeasurement>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PRONTO_MSGS__MSG__DETAIL__INDEXED_MEASUREMENT__TRAITS_HPP_
