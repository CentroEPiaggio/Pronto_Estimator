// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from velocity_command_msgs:msg/SimpleVelocityCommand.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "velocity_command_msgs/msg/detail/simple_velocity_command__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace velocity_command_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void SimpleVelocityCommand_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) velocity_command_msgs::msg::SimpleVelocityCommand(_init);
}

void SimpleVelocityCommand_fini_function(void * message_memory)
{
  auto typed_message = static_cast<velocity_command_msgs::msg::SimpleVelocityCommand *>(message_memory);
  typed_message->~SimpleVelocityCommand();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SimpleVelocityCommand_message_member_array[3] = {
  {
    "velocity_forward",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(velocity_command_msgs::msg::SimpleVelocityCommand, velocity_forward),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "velocity_lateral",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(velocity_command_msgs::msg::SimpleVelocityCommand, velocity_lateral),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "yaw_rate",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(velocity_command_msgs::msg::SimpleVelocityCommand, yaw_rate),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SimpleVelocityCommand_message_members = {
  "velocity_command_msgs::msg",  // message namespace
  "SimpleVelocityCommand",  // message name
  3,  // number of fields
  sizeof(velocity_command_msgs::msg::SimpleVelocityCommand),
  SimpleVelocityCommand_message_member_array,  // message members
  SimpleVelocityCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  SimpleVelocityCommand_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SimpleVelocityCommand_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SimpleVelocityCommand_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace velocity_command_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<velocity_command_msgs::msg::SimpleVelocityCommand>()
{
  return &::velocity_command_msgs::msg::rosidl_typesupport_introspection_cpp::SimpleVelocityCommand_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, velocity_command_msgs, msg, SimpleVelocityCommand)() {
  return &::velocity_command_msgs::msg::rosidl_typesupport_introspection_cpp::SimpleVelocityCommand_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
