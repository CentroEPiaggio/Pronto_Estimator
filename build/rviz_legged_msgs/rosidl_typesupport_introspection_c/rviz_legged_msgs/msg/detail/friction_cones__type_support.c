// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rviz_legged_msgs:msg/FrictionCones.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rviz_legged_msgs/msg/detail/friction_cones__rosidl_typesupport_introspection_c.h"
#include "rviz_legged_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rviz_legged_msgs/msg/detail/friction_cones__functions.h"
#include "rviz_legged_msgs/msg/detail/friction_cones__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `friction_cones`
#include "rviz_legged_msgs/msg/friction_cone.h"
// Member `friction_cones`
#include "rviz_legged_msgs/msg/detail/friction_cone__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rviz_legged_msgs__msg__FrictionCones__rosidl_typesupport_introspection_c__FrictionCones_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rviz_legged_msgs__msg__FrictionCones__init(message_memory);
}

void rviz_legged_msgs__msg__FrictionCones__rosidl_typesupport_introspection_c__FrictionCones_fini_function(void * message_memory)
{
  rviz_legged_msgs__msg__FrictionCones__fini(message_memory);
}

size_t rviz_legged_msgs__msg__FrictionCones__rosidl_typesupport_introspection_c__size_function__FrictionCones__friction_cones(
  const void * untyped_member)
{
  const rviz_legged_msgs__msg__FrictionCone__Sequence * member =
    (const rviz_legged_msgs__msg__FrictionCone__Sequence *)(untyped_member);
  return member->size;
}

const void * rviz_legged_msgs__msg__FrictionCones__rosidl_typesupport_introspection_c__get_const_function__FrictionCones__friction_cones(
  const void * untyped_member, size_t index)
{
  const rviz_legged_msgs__msg__FrictionCone__Sequence * member =
    (const rviz_legged_msgs__msg__FrictionCone__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rviz_legged_msgs__msg__FrictionCones__rosidl_typesupport_introspection_c__get_function__FrictionCones__friction_cones(
  void * untyped_member, size_t index)
{
  rviz_legged_msgs__msg__FrictionCone__Sequence * member =
    (rviz_legged_msgs__msg__FrictionCone__Sequence *)(untyped_member);
  return &member->data[index];
}

void rviz_legged_msgs__msg__FrictionCones__rosidl_typesupport_introspection_c__fetch_function__FrictionCones__friction_cones(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rviz_legged_msgs__msg__FrictionCone * item =
    ((const rviz_legged_msgs__msg__FrictionCone *)
    rviz_legged_msgs__msg__FrictionCones__rosidl_typesupport_introspection_c__get_const_function__FrictionCones__friction_cones(untyped_member, index));
  rviz_legged_msgs__msg__FrictionCone * value =
    (rviz_legged_msgs__msg__FrictionCone *)(untyped_value);
  *value = *item;
}

void rviz_legged_msgs__msg__FrictionCones__rosidl_typesupport_introspection_c__assign_function__FrictionCones__friction_cones(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rviz_legged_msgs__msg__FrictionCone * item =
    ((rviz_legged_msgs__msg__FrictionCone *)
    rviz_legged_msgs__msg__FrictionCones__rosidl_typesupport_introspection_c__get_function__FrictionCones__friction_cones(untyped_member, index));
  const rviz_legged_msgs__msg__FrictionCone * value =
    (const rviz_legged_msgs__msg__FrictionCone *)(untyped_value);
  *item = *value;
}

bool rviz_legged_msgs__msg__FrictionCones__rosidl_typesupport_introspection_c__resize_function__FrictionCones__friction_cones(
  void * untyped_member, size_t size)
{
  rviz_legged_msgs__msg__FrictionCone__Sequence * member =
    (rviz_legged_msgs__msg__FrictionCone__Sequence *)(untyped_member);
  rviz_legged_msgs__msg__FrictionCone__Sequence__fini(member);
  return rviz_legged_msgs__msg__FrictionCone__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember rviz_legged_msgs__msg__FrictionCones__rosidl_typesupport_introspection_c__FrictionCones_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rviz_legged_msgs__msg__FrictionCones, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "friction_cones",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rviz_legged_msgs__msg__FrictionCones, friction_cones),  // bytes offset in struct
    NULL,  // default value
    rviz_legged_msgs__msg__FrictionCones__rosidl_typesupport_introspection_c__size_function__FrictionCones__friction_cones,  // size() function pointer
    rviz_legged_msgs__msg__FrictionCones__rosidl_typesupport_introspection_c__get_const_function__FrictionCones__friction_cones,  // get_const(index) function pointer
    rviz_legged_msgs__msg__FrictionCones__rosidl_typesupport_introspection_c__get_function__FrictionCones__friction_cones,  // get(index) function pointer
    rviz_legged_msgs__msg__FrictionCones__rosidl_typesupport_introspection_c__fetch_function__FrictionCones__friction_cones,  // fetch(index, &value) function pointer
    rviz_legged_msgs__msg__FrictionCones__rosidl_typesupport_introspection_c__assign_function__FrictionCones__friction_cones,  // assign(index, value) function pointer
    rviz_legged_msgs__msg__FrictionCones__rosidl_typesupport_introspection_c__resize_function__FrictionCones__friction_cones  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rviz_legged_msgs__msg__FrictionCones__rosidl_typesupport_introspection_c__FrictionCones_message_members = {
  "rviz_legged_msgs__msg",  // message namespace
  "FrictionCones",  // message name
  2,  // number of fields
  sizeof(rviz_legged_msgs__msg__FrictionCones),
  rviz_legged_msgs__msg__FrictionCones__rosidl_typesupport_introspection_c__FrictionCones_message_member_array,  // message members
  rviz_legged_msgs__msg__FrictionCones__rosidl_typesupport_introspection_c__FrictionCones_init_function,  // function to initialize message memory (memory has to be allocated)
  rviz_legged_msgs__msg__FrictionCones__rosidl_typesupport_introspection_c__FrictionCones_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rviz_legged_msgs__msg__FrictionCones__rosidl_typesupport_introspection_c__FrictionCones_message_type_support_handle = {
  0,
  &rviz_legged_msgs__msg__FrictionCones__rosidl_typesupport_introspection_c__FrictionCones_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rviz_legged_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rviz_legged_msgs, msg, FrictionCones)() {
  rviz_legged_msgs__msg__FrictionCones__rosidl_typesupport_introspection_c__FrictionCones_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  rviz_legged_msgs__msg__FrictionCones__rosidl_typesupport_introspection_c__FrictionCones_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rviz_legged_msgs, msg, FrictionCone)();
  if (!rviz_legged_msgs__msg__FrictionCones__rosidl_typesupport_introspection_c__FrictionCones_message_type_support_handle.typesupport_identifier) {
    rviz_legged_msgs__msg__FrictionCones__rosidl_typesupport_introspection_c__FrictionCones_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rviz_legged_msgs__msg__FrictionCones__rosidl_typesupport_introspection_c__FrictionCones_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif