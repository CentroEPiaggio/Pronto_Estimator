// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from pronto_msgs:msg/IndexedMeasurement.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "pronto_msgs/msg/detail/indexed_measurement__rosidl_typesupport_introspection_c.h"
#include "pronto_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "pronto_msgs/msg/detail/indexed_measurement__functions.h"
#include "pronto_msgs/msg/detail/indexed_measurement__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `z_effective`
// Member `z_indices`
// Member `r_effective`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void IndexedMeasurement__rosidl_typesupport_introspection_c__IndexedMeasurement_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pronto_msgs__msg__IndexedMeasurement__init(message_memory);
}

void IndexedMeasurement__rosidl_typesupport_introspection_c__IndexedMeasurement_fini_function(void * message_memory)
{
  pronto_msgs__msg__IndexedMeasurement__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember IndexedMeasurement__rosidl_typesupport_introspection_c__IndexedMeasurement_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pronto_msgs__msg__IndexedMeasurement, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "utime",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pronto_msgs__msg__IndexedMeasurement, utime),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "state_utime",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pronto_msgs__msg__IndexedMeasurement, state_utime),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "z_effective",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pronto_msgs__msg__IndexedMeasurement, z_effective),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "z_indices",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pronto_msgs__msg__IndexedMeasurement, z_indices),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "r_effective",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pronto_msgs__msg__IndexedMeasurement, r_effective),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers IndexedMeasurement__rosidl_typesupport_introspection_c__IndexedMeasurement_message_members = {
  "pronto_msgs__msg",  // message namespace
  "IndexedMeasurement",  // message name
  6,  // number of fields
  sizeof(pronto_msgs__msg__IndexedMeasurement),
  IndexedMeasurement__rosidl_typesupport_introspection_c__IndexedMeasurement_message_member_array,  // message members
  IndexedMeasurement__rosidl_typesupport_introspection_c__IndexedMeasurement_init_function,  // function to initialize message memory (memory has to be allocated)
  IndexedMeasurement__rosidl_typesupport_introspection_c__IndexedMeasurement_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t IndexedMeasurement__rosidl_typesupport_introspection_c__IndexedMeasurement_message_type_support_handle = {
  0,
  &IndexedMeasurement__rosidl_typesupport_introspection_c__IndexedMeasurement_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pronto_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pronto_msgs, msg, IndexedMeasurement)() {
  IndexedMeasurement__rosidl_typesupport_introspection_c__IndexedMeasurement_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!IndexedMeasurement__rosidl_typesupport_introspection_c__IndexedMeasurement_message_type_support_handle.typesupport_identifier) {
    IndexedMeasurement__rosidl_typesupport_introspection_c__IndexedMeasurement_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &IndexedMeasurement__rosidl_typesupport_introspection_c__IndexedMeasurement_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
