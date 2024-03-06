// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from generalized_pose_msgs:msg/GeneralizedPose.idl
// generated code does not contain a copyright notice

#ifndef GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSE__TRAITS_HPP_
#define GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "generalized_pose_msgs/msg/detail/generalized_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'base_acc'
// Member 'base_vel'
// Member 'base_pos'
// Member 'base_angvel'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"
// Member 'base_quat'
#include "geometry_msgs/msg/detail/quaternion__traits.hpp"

namespace generalized_pose_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const GeneralizedPose & msg,
  std::ostream & out)
{
  out << "{";
  // member: base_acc
  {
    out << "base_acc: ";
    to_flow_style_yaml(msg.base_acc, out);
    out << ", ";
  }

  // member: base_vel
  {
    out << "base_vel: ";
    to_flow_style_yaml(msg.base_vel, out);
    out << ", ";
  }

  // member: base_pos
  {
    out << "base_pos: ";
    to_flow_style_yaml(msg.base_pos, out);
    out << ", ";
  }

  // member: base_angvel
  {
    out << "base_angvel: ";
    to_flow_style_yaml(msg.base_angvel, out);
    out << ", ";
  }

  // member: base_quat
  {
    out << "base_quat: ";
    to_flow_style_yaml(msg.base_quat, out);
    out << ", ";
  }

  // member: feet_acc
  {
    if (msg.feet_acc.size() == 0) {
      out << "feet_acc: []";
    } else {
      out << "feet_acc: [";
      size_t pending_items = msg.feet_acc.size();
      for (auto item : msg.feet_acc) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: feet_vel
  {
    if (msg.feet_vel.size() == 0) {
      out << "feet_vel: []";
    } else {
      out << "feet_vel: [";
      size_t pending_items = msg.feet_vel.size();
      for (auto item : msg.feet_vel) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: feet_pos
  {
    if (msg.feet_pos.size() == 0) {
      out << "feet_pos: []";
    } else {
      out << "feet_pos: [";
      size_t pending_items = msg.feet_pos.size();
      for (auto item : msg.feet_pos) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: contact_feet
  {
    if (msg.contact_feet.size() == 0) {
      out << "contact_feet: []";
    } else {
      out << "contact_feet: [";
      size_t pending_items = msg.contact_feet.size();
      for (auto item : msg.contact_feet) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const GeneralizedPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: base_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "base_acc:\n";
    to_block_style_yaml(msg.base_acc, out, indentation + 2);
  }

  // member: base_vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "base_vel:\n";
    to_block_style_yaml(msg.base_vel, out, indentation + 2);
  }

  // member: base_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "base_pos:\n";
    to_block_style_yaml(msg.base_pos, out, indentation + 2);
  }

  // member: base_angvel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "base_angvel:\n";
    to_block_style_yaml(msg.base_angvel, out, indentation + 2);
  }

  // member: base_quat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "base_quat:\n";
    to_block_style_yaml(msg.base_quat, out, indentation + 2);
  }

  // member: feet_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.feet_acc.size() == 0) {
      out << "feet_acc: []\n";
    } else {
      out << "feet_acc:\n";
      for (auto item : msg.feet_acc) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: feet_vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.feet_vel.size() == 0) {
      out << "feet_vel: []\n";
    } else {
      out << "feet_vel:\n";
      for (auto item : msg.feet_vel) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: feet_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.feet_pos.size() == 0) {
      out << "feet_pos: []\n";
    } else {
      out << "feet_pos:\n";
      for (auto item : msg.feet_pos) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: contact_feet
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.contact_feet.size() == 0) {
      out << "contact_feet: []\n";
    } else {
      out << "contact_feet:\n";
      for (auto item : msg.contact_feet) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GeneralizedPose & msg, bool use_flow_style = false)
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
  const generalized_pose_msgs::msg::GeneralizedPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  generalized_pose_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use generalized_pose_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const generalized_pose_msgs::msg::GeneralizedPose & msg)
{
  return generalized_pose_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<generalized_pose_msgs::msg::GeneralizedPose>()
{
  return "generalized_pose_msgs::msg::GeneralizedPose";
}

template<>
inline const char * name<generalized_pose_msgs::msg::GeneralizedPose>()
{
  return "generalized_pose_msgs/msg/GeneralizedPose";
}

template<>
struct has_fixed_size<generalized_pose_msgs::msg::GeneralizedPose>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<generalized_pose_msgs::msg::GeneralizedPose>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<generalized_pose_msgs::msg::GeneralizedPose>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSE__TRAITS_HPP_
