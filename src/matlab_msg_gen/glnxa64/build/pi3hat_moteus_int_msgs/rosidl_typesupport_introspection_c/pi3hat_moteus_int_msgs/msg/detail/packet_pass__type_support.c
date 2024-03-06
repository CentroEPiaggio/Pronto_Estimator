// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from pi3hat_moteus_int_msgs:msg/PacketPass.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "pi3hat_moteus_int_msgs/msg/detail/packet_pass__rosidl_typesupport_introspection_c.h"
#include "pi3hat_moteus_int_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "pi3hat_moteus_int_msgs/msg/detail/packet_pass__functions.h"
#include "pi3hat_moteus_int_msgs/msg/detail/packet_pass__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `pack_loss`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void PacketPass__rosidl_typesupport_introspection_c__PacketPass_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pi3hat_moteus_int_msgs__msg__PacketPass__init(message_memory);
}

void PacketPass__rosidl_typesupport_introspection_c__PacketPass_fini_function(void * message_memory)
{
  pi3hat_moteus_int_msgs__msg__PacketPass__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember PacketPass__rosidl_typesupport_introspection_c__PacketPass_message_member_array[5] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pi3hat_moteus_int_msgs__msg__PacketPass, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "valid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pi3hat_moteus_int_msgs__msg__PacketPass, valid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cycle_dur",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pi3hat_moteus_int_msgs__msg__PacketPass, cycle_dur),  // bytes offset in struct
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
    offsetof(pi3hat_moteus_int_msgs__msg__PacketPass, name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pack_loss",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pi3hat_moteus_int_msgs__msg__PacketPass, pack_loss),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers PacketPass__rosidl_typesupport_introspection_c__PacketPass_message_members = {
  "pi3hat_moteus_int_msgs__msg",  // message namespace
  "PacketPass",  // message name
  5,  // number of fields
  sizeof(pi3hat_moteus_int_msgs__msg__PacketPass),
  PacketPass__rosidl_typesupport_introspection_c__PacketPass_message_member_array,  // message members
  PacketPass__rosidl_typesupport_introspection_c__PacketPass_init_function,  // function to initialize message memory (memory has to be allocated)
  PacketPass__rosidl_typesupport_introspection_c__PacketPass_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t PacketPass__rosidl_typesupport_introspection_c__PacketPass_message_type_support_handle = {
  0,
  &PacketPass__rosidl_typesupport_introspection_c__PacketPass_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pi3hat_moteus_int_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pi3hat_moteus_int_msgs, msg, PacketPass)() {
  PacketPass__rosidl_typesupport_introspection_c__PacketPass_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!PacketPass__rosidl_typesupport_introspection_c__PacketPass_message_type_support_handle.typesupport_identifier) {
    PacketPass__rosidl_typesupport_introspection_c__PacketPass_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &PacketPass__rosidl_typesupport_introspection_c__PacketPass_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
