// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from generalized_pose_msgs:msg/GeneralizedPosesWithTime.idl
// generated code does not contain a copyright notice

#ifndef GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSES_WITH_TIME__TRAITS_HPP_
#define GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSES_WITH_TIME__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "generalized_pose_msgs/msg/detail/generalized_poses_with_time__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'generalized_poses_with_time'
#include "generalized_pose_msgs/msg/detail/generalized_pose_with_time__traits.hpp"

namespace generalized_pose_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const GeneralizedPosesWithTime & msg,
  std::ostream & out)
{
  out << "{";
  // member: generalized_poses_with_time
  {
    if (msg.generalized_poses_with_time.size() == 0) {
      out << "generalized_poses_with_time: []";
    } else {
      out << "generalized_poses_with_time: [";
      size_t pending_items = msg.generalized_poses_with_time.size();
      for (auto item : msg.generalized_poses_with_time) {
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
  const GeneralizedPosesWithTime & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: generalized_poses_with_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.generalized_poses_with_time.size() == 0) {
      out << "generalized_poses_with_time: []\n";
    } else {
      out << "generalized_poses_with_time:\n";
      for (auto item : msg.generalized_poses_with_time) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GeneralizedPosesWithTime & msg, bool use_flow_style = false)
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
  const generalized_pose_msgs::msg::GeneralizedPosesWithTime & msg,
  std::ostream & out, size_t indentation = 0)
{
  generalized_pose_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use generalized_pose_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const generalized_pose_msgs::msg::GeneralizedPosesWithTime & msg)
{
  return generalized_pose_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<generalized_pose_msgs::msg::GeneralizedPosesWithTime>()
{
  return "generalized_pose_msgs::msg::GeneralizedPosesWithTime";
}

template<>
inline const char * name<generalized_pose_msgs::msg::GeneralizedPosesWithTime>()
{
  return "generalized_pose_msgs/msg/GeneralizedPosesWithTime";
}

template<>
struct has_fixed_size<generalized_pose_msgs::msg::GeneralizedPosesWithTime>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<generalized_pose_msgs::msg::GeneralizedPosesWithTime>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<generalized_pose_msgs::msg::GeneralizedPosesWithTime>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSES_WITH_TIME__TRAITS_HPP_
