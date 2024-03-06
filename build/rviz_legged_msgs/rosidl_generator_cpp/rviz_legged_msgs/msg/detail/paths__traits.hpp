// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rviz_legged_msgs:msg/Paths.idl
// generated code does not contain a copyright notice

#ifndef RVIZ_LEGGED_MSGS__MSG__DETAIL__PATHS__TRAITS_HPP_
#define RVIZ_LEGGED_MSGS__MSG__DETAIL__PATHS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rviz_legged_msgs/msg/detail/paths__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'paths'
#include "nav_msgs/msg/detail/path__traits.hpp"

namespace rviz_legged_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Paths & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: paths
  {
    if (msg.paths.size() == 0) {
      out << "paths: []";
    } else {
      out << "paths: [";
      size_t pending_items = msg.paths.size();
      for (auto item : msg.paths) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Paths & msg,
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

  // member: paths
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.paths.size() == 0) {
      out << "paths: []\n";
    } else {
      out << "paths:\n";
      for (auto item : msg.paths) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Paths & msg, bool use_flow_style = false)
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
  const rviz_legged_msgs::msg::Paths & msg,
  std::ostream & out, size_t indentation = 0)
{
  rviz_legged_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rviz_legged_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const rviz_legged_msgs::msg::Paths & msg)
{
  return rviz_legged_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rviz_legged_msgs::msg::Paths>()
{
  return "rviz_legged_msgs::msg::Paths";
}

template<>
inline const char * name<rviz_legged_msgs::msg::Paths>()
{
  return "rviz_legged_msgs/msg/Paths";
}

template<>
struct has_fixed_size<rviz_legged_msgs::msg::Paths>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rviz_legged_msgs::msg::Paths>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rviz_legged_msgs::msg::Paths>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RVIZ_LEGGED_MSGS__MSG__DETAIL__PATHS__TRAITS_HPP_
