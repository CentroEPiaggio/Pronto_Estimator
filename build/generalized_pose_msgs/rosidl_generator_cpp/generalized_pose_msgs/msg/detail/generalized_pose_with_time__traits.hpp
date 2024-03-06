// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from generalized_pose_msgs:msg/GeneralizedPoseWithTime.idl
// generated code does not contain a copyright notice

#ifndef GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSE_WITH_TIME__TRAITS_HPP_
#define GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSE_WITH_TIME__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "generalized_pose_msgs/msg/detail/generalized_pose_with_time__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'generalized_pose'
#include "generalized_pose_msgs/msg/detail/generalized_pose__traits.hpp"

namespace generalized_pose_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const GeneralizedPoseWithTime & msg,
  std::ostream & out)
{
  out << "{";
  // member: generalized_pose
  {
    out << "generalized_pose: ";
    to_flow_style_yaml(msg.generalized_pose, out);
    out << ", ";
  }

  // member: time
  {
    out << "time: ";
    rosidl_generator_traits::value_to_yaml(msg.time, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GeneralizedPoseWithTime & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: generalized_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "generalized_pose:\n";
    to_block_style_yaml(msg.generalized_pose, out, indentation + 2);
  }

  // member: time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time: ";
    rosidl_generator_traits::value_to_yaml(msg.time, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GeneralizedPoseWithTime & msg, bool use_flow_style = false)
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

}  // namespace generalized_pose_msgs

namespace rosidl_generator_traits
{

[[deprecated("use generalized_pose_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const generalized_pose_msgs::msg::GeneralizedPoseWithTime & msg,
  std::ostream & out, size_t indentation = 0)
{
  generalized_pose_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use generalized_pose_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const generalized_pose_msgs::msg::GeneralizedPoseWithTime & msg)
{
  return generalized_pose_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<generalized_pose_msgs::msg::GeneralizedPoseWithTime>()
{
  return "generalized_pose_msgs::msg::GeneralizedPoseWithTime";
}

template<>
inline const char * name<generalized_pose_msgs::msg::GeneralizedPoseWithTime>()
{
  return "generalized_pose_msgs/msg/GeneralizedPoseWithTime";
}

template<>
struct has_fixed_size<generalized_pose_msgs::msg::GeneralizedPoseWithTime>
  : std::integral_constant<bool, has_fixed_size<generalized_pose_msgs::msg::GeneralizedPose>::value> {};

template<>
struct has_bounded_size<generalized_pose_msgs::msg::GeneralizedPoseWithTime>
  : std::integral_constant<bool, has_bounded_size<generalized_pose_msgs::msg::GeneralizedPose>::value> {};

template<>
struct is_message<generalized_pose_msgs::msg::GeneralizedPoseWithTime>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSE_WITH_TIME__TRAITS_HPP_
