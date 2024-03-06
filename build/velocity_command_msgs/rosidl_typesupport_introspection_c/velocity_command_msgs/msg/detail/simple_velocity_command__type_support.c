// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from velocity_command_msgs:msg/SimpleVelocityCommand.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "velocity_command_msgs/msg/detail/simple_velocity_command__rosidl_typesupport_introspection_c.h"
#include "velocity_command_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "velocity_command_msgs/msg/detail/simple_velocity_command__functions.h"
#include "velocity_command_msgs/msg/detail/simple_velocity_command__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void velocity_command_msgs__msg__SimpleVelocityCommand__rosidl_typesupport_introspection_c__SimpleVelocityCommand_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  velocity_command_msgs__msg__SimpleVelocityCommand__init(message_memory);
}

void velocity_command_msgs__msg__SimpleVelocityCommand__rosidl_typesupport_introspection_c__SimpleVelocityCommand_fini_function(void * message_memory)
{
  velocity_command_msgs__msg__SimpleVelocityCommand__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember velocity_command_msgs__msg__SimpleVelocityCommand__rosidl_typesupport_introspection_c__SimpleVelocityCommand_message_member_array[3] = {
  {
    "velocity_forward",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(velocity_command_msgs__msg__SimpleVelocityCommand, velocity_forward),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity_lateral",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(velocity_command_msgs__msg__SimpleVelocityCommand, velocity_lateral),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "yaw_rate",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(velocity_command_msgs__msg__SimpleVelocityCommand, yaw_rate),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers velocity_command_msgs__msg__SimpleVelocityCommand__rosidl_typesupport_introspection_c__SimpleVelocityCommand_message_members = {
  "velocity_command_msgs__msg",  // message namespace
  "SimpleVelocityCommand",  // message name
  3,  // number of fields
  sizeof(velocity_command_msgs__msg__SimpleVelocityCommand),
  velocity_command_msgs__msg__SimpleVelocityCommand__rosidl_typesupport_introspection_c__SimpleVelocityCommand_message_member_array,  // message members
  velocity_command_msgs__msg__SimpleVelocityCommand__rosidl_typesupport_introspection_c__SimpleVelocityCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  velocity_command_msgs__msg__SimpleVelocityCommand__rosidl_typesupport_introspection_c__SimpleVelocityCommand_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t velocity_command_msgs__msg__SimpleVelocityCommand__rosidl_typesupport_introspection_c__SimpleVelocityCommand_message_type_support_handle = {
  0,
  &velocity_command_msgs__msg__SimpleVelocityCommand__rosidl_typesupport_introspection_c__SimpleVelocityCommand_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_velocity_command_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, velocity_command_msgs, msg, SimpleVelocityCommand)() {
  if (!velocity_command_msgs__msg__SimpleVelocityCommand__rosidl_typesupport_introspection_c__SimpleVelocityCommand_message_type_support_handle.typesupport_identifier) {
    velocity_command_msgs__msg__SimpleVelocityCommand__rosidl_typesupport_introspection_c__SimpleVelocityCommand_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &velocity_command_msgs__msg__SimpleVelocityCommand__rosidl_typesupport_introspection_c__SimpleVelocityCommand_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
