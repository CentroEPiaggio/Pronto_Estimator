// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from pronto_msgs:msg/VisualOdometryUpdate.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "pronto_msgs/msg/detail/visual_odometry_update__rosidl_typesupport_introspection_c.h"
#include "pronto_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "pronto_msgs/msg/detail/visual_odometry_update__functions.h"
#include "pronto_msgs/msg/detail/visual_odometry_update__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `curr_timestamp`
// Member `prev_timestamp`
#include "builtin_interfaces/msg/time.h"
// Member `curr_timestamp`
// Member `prev_timestamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"
// Member `relative_transform`
#include "geometry_msgs/msg/transform.h"
// Member `relative_transform`
#include "geometry_msgs/msg/detail/transform__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void VisualOdometryUpdate__rosidl_typesupport_introspection_c__VisualOdometryUpdate_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pronto_msgs__msg__VisualOdometryUpdate__init(message_memory);
}

void VisualOdometryUpdate__rosidl_typesupport_introspection_c__VisualOdometryUpdate_fini_function(void * message_memory)
{
  pronto_msgs__msg__VisualOdometryUpdate__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember VisualOdometryUpdate__rosidl_typesupport_introspection_c__VisualOdometryUpdate_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pronto_msgs__msg__VisualOdometryUpdate, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "curr_timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pronto_msgs__msg__VisualOdometryUpdate, curr_timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "prev_timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pronto_msgs__msg__VisualOdometryUpdate, prev_timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "relative_transform",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pronto_msgs__msg__VisualOdometryUpdate, relative_transform),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "covariance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    36,  // array size
    false,  // is upper bound
    offsetof(pronto_msgs__msg__VisualOdometryUpdate, covariance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "estimate_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pronto_msgs__msg__VisualOdometryUpdate, estimate_status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers VisualOdometryUpdate__rosidl_typesupport_introspection_c__VisualOdometryUpdate_message_members = {
  "pronto_msgs__msg",  // message namespace
  "VisualOdometryUpdate",  // message name
  6,  // number of fields
  sizeof(pronto_msgs__msg__VisualOdometryUpdate),
  VisualOdometryUpdate__rosidl_typesupport_introspection_c__VisualOdometryUpdate_message_member_array,  // message members
  VisualOdometryUpdate__rosidl_typesupport_introspection_c__VisualOdometryUpdate_init_function,  // function to initialize message memory (memory has to be allocated)
  VisualOdometryUpdate__rosidl_typesupport_introspection_c__VisualOdometryUpdate_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t VisualOdometryUpdate__rosidl_typesupport_introspection_c__VisualOdometryUpdate_message_type_support_handle = {
  0,
  &VisualOdometryUpdate__rosidl_typesupport_introspection_c__VisualOdometryUpdate_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pronto_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pronto_msgs, msg, VisualOdometryUpdate)() {
  VisualOdometryUpdate__rosidl_typesupport_introspection_c__VisualOdometryUpdate_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  VisualOdometryUpdate__rosidl_typesupport_introspection_c__VisualOdometryUpdate_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  VisualOdometryUpdate__rosidl_typesupport_introspection_c__VisualOdometryUpdate_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  VisualOdometryUpdate__rosidl_typesupport_introspection_c__VisualOdometryUpdate_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Transform)();
  if (!VisualOdometryUpdate__rosidl_typesupport_introspection_c__VisualOdometryUpdate_message_type_support_handle.typesupport_identifier) {
    VisualOdometryUpdate__rosidl_typesupport_introspection_c__VisualOdometryUpdate_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &VisualOdometryUpdate__rosidl_typesupport_introspection_c__VisualOdometryUpdate_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif