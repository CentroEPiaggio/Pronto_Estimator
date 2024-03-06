// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rviz_legged_msgs:msg/FrictionCone.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rviz_legged_msgs/msg/detail/friction_cone__rosidl_typesupport_introspection_c.h"
#include "rviz_legged_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rviz_legged_msgs/msg/detail/friction_cone__functions.h"
#include "rviz_legged_msgs/msg/detail/friction_cone__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `normal_direction`
#include "geometry_msgs/msg/vector3.h"
// Member `normal_direction`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rviz_legged_msgs__msg__FrictionCone__rosidl_typesupport_introspection_c__FrictionCone_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rviz_legged_msgs__msg__FrictionCone__init(message_memory);
}

void rviz_legged_msgs__msg__FrictionCone__rosidl_typesupport_introspection_c__FrictionCone_fini_function(void * message_memory)
{
  rviz_legged_msgs__msg__FrictionCone__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rviz_legged_msgs__msg__FrictionCone__rosidl_typesupport_introspection_c__FrictionCone_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rviz_legged_msgs__msg__FrictionCone, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "normal_direction",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rviz_legged_msgs__msg__FrictionCone, normal_direction),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "friction_coefficient",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rviz_legged_msgs__msg__FrictionCone, friction_coefficient),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rviz_legged_msgs__msg__FrictionCone__rosidl_typesupport_introspection_c__FrictionCone_message_members = {
  "rviz_legged_msgs__msg",  // message namespace
  "FrictionCone",  // message name
  3,  // number of fields
  sizeof(rviz_legged_msgs__msg__FrictionCone),
  rviz_legged_msgs__msg__FrictionCone__rosidl_typesupport_introspection_c__FrictionCone_message_member_array,  // message members
  rviz_legged_msgs__msg__FrictionCone__rosidl_typesupport_introspection_c__FrictionCone_init_function,  // function to initialize message memory (memory has to be allocated)
  rviz_legged_msgs__msg__FrictionCone__rosidl_typesupport_introspection_c__FrictionCone_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rviz_legged_msgs__msg__FrictionCone__rosidl_typesupport_introspection_c__FrictionCone_message_type_support_handle = {
  0,
  &rviz_legged_msgs__msg__FrictionCone__rosidl_typesupport_introspection_c__FrictionCone_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rviz_legged_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rviz_legged_msgs, msg, FrictionCone)() {
  rviz_legged_msgs__msg__FrictionCone__rosidl_typesupport_introspection_c__FrictionCone_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  rviz_legged_msgs__msg__FrictionCone__rosidl_typesupport_introspection_c__FrictionCone_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  if (!rviz_legged_msgs__msg__FrictionCone__rosidl_typesupport_introspection_c__FrictionCone_message_type_support_handle.typesupport_identifier) {
    rviz_legged_msgs__msg__FrictionCone__rosidl_typesupport_introspection_c__FrictionCone_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rviz_legged_msgs__msg__FrictionCone__rosidl_typesupport_introspection_c__FrictionCone_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
