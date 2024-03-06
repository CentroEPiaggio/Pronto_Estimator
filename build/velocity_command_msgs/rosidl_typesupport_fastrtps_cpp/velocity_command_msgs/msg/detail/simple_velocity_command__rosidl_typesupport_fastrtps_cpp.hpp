// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from velocity_command_msgs:msg/SimpleVelocityCommand.idl
// generated code does not contain a copyright notice

#ifndef VELOCITY_COMMAND_MSGS__MSG__DETAIL__SIMPLE_VELOCITY_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define VELOCITY_COMMAND_MSGS__MSG__DETAIL__SIMPLE_VELOCITY_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "velocity_command_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "velocity_command_msgs/msg/detail/simple_velocity_command__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace velocity_command_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_velocity_command_msgs
cdr_serialize(
  const velocity_command_msgs::msg::SimpleVelocityCommand & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_velocity_command_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  velocity_command_msgs::msg::SimpleVelocityCommand & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_velocity_command_msgs
get_serialized_size(
  const velocity_command_msgs::msg::SimpleVelocityCommand & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_velocity_command_msgs
max_serialized_size_SimpleVelocityCommand(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace velocity_command_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_velocity_command_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, velocity_command_msgs, msg, SimpleVelocityCommand)();

#ifdef __cplusplus
}
#endif

#endif  // VELOCITY_COMMAND_MSGS__MSG__DETAIL__SIMPLE_VELOCITY_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
