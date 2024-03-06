// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pi3hat_moteus_int_msgs:msg/OmniMulinexCommand.idl
// generated code does not contain a copyright notice

#ifndef PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__OMNI_MULINEX_COMMAND__TRAITS_HPP_
#define PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__OMNI_MULINEX_COMMAND__TRAITS_HPP_

#include "pi3hat_moteus_int_msgs/msg/detail/omni_mulinex_command__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<pi3hat_moteus_int_msgs::msg::OmniMulinexCommand>()
{
  return "pi3hat_moteus_int_msgs::msg::OmniMulinexCommand";
}

template<>
inline const char * name<pi3hat_moteus_int_msgs::msg::OmniMulinexCommand>()
{
  return "pi3hat_moteus_int_msgs/msg/OmniMulinexCommand";
}

template<>
struct has_fixed_size<pi3hat_moteus_int_msgs::msg::OmniMulinexCommand>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<pi3hat_moteus_int_msgs::msg::OmniMulinexCommand>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<pi3hat_moteus_int_msgs::msg::OmniMulinexCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__OMNI_MULINEX_COMMAND__TRAITS_HPP_
