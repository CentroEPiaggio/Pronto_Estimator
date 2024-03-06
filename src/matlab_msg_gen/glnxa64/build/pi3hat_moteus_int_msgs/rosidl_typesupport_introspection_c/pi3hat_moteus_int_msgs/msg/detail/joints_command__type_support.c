// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from pi3hat_moteus_int_msgs:msg/JointsCommand.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "pi3hat_moteus_int_msgs/msg/detail/joints_command__rosidl_typesupport_introspection_c.h"
#include "pi3hat_moteus_int_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "pi3hat_moteus_int_msgs/msg/detail/joints_command__functions.h"
#include "pi3hat_moteus_int_msgs/msg/detail/joints_command__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `position`
// Member `velocity`
// Member `effort`
// Member `kp_scale`
// Member `kd_scale`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void JointsCommand__rosidl_typesupport_introspection_c__JointsCommand_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pi3hat_moteus_int_msgs__msg__JointsCommand__init(message_memory);
}

void JointsCommand__rosidl_typesupport_introspection_c__JointsCommand_fini_function(void * message_memory)
{
  pi3hat_moteus_int_msgs__msg__JointsCommand__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember JointsCommand__rosidl_typesupport_introspection_c__JointsCommand_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pi3hat_moteus_int_msgs__msg__JointsCommand, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pi3hat_moteus_int_msgs__msg__JointsCommand, name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pi3hat_moteus_int_msgs__msg__JointsCommand, position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pi3hat_moteus_int_msgs__msg__JointsCommand, velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "effort",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pi3hat_moteus_int_msgs__msg__JointsCommand, effort),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "kp_scale",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pi3hat_moteus_int_msgs__msg__JointsCommand, kp_scale),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "kd_scale",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pi3hat_moteus_int_msgs__msg__JointsCommand, kd_scale),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers JointsCommand__rosidl_typesupport_introspection_c__JointsCommand_message_members = {
  "pi3hat_moteus_int_msgs__msg",  // message namespace
  "JointsCommand",  // message name
  7,  // number of fields
  sizeof(pi3hat_moteus_int_msgs__msg__JointsCommand),
  JointsCommand__rosidl_typesupport_introspection_c__JointsCommand_message_member_array,  // message members
  JointsCommand__rosidl_typesupport_introspection_c__JointsCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  JointsCommand__rosidl_typesupport_introspection_c__JointsCommand_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t JointsCommand__rosidl_typesupport_introspection_c__JointsCommand_message_type_support_handle = {
  0,
  &JointsCommand__rosidl_typesupport_introspection_c__JointsCommand_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pi3hat_moteus_int_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pi3hat_moteus_int_msgs, msg, JointsCommand)() {
  JointsCommand__rosidl_typesupport_introspection_c__JointsCommand_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!JointsCommand__rosidl_typesupport_introspection_c__JointsCommand_message_type_support_handle.typesupport_identifier) {
    JointsCommand__rosidl_typesupport_introspection_c__JointsCommand_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &JointsCommand__rosidl_typesupport_introspection_c__JointsCommand_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
