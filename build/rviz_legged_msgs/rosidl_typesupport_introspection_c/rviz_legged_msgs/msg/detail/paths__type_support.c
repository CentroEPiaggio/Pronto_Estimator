// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rviz_legged_msgs:msg/Paths.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rviz_legged_msgs/msg/detail/paths__rosidl_typesupport_introspection_c.h"
#include "rviz_legged_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rviz_legged_msgs/msg/detail/paths__functions.h"
#include "rviz_legged_msgs/msg/detail/paths__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `paths`
#include "nav_msgs/msg/path.h"
// Member `paths`
#include "nav_msgs/msg/detail/path__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rviz_legged_msgs__msg__Paths__rosidl_typesupport_introspection_c__Paths_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rviz_legged_msgs__msg__Paths__init(message_memory);
}

void rviz_legged_msgs__msg__Paths__rosidl_typesupport_introspection_c__Paths_fini_function(void * message_memory)
{
  rviz_legged_msgs__msg__Paths__fini(message_memory);
}

size_t rviz_legged_msgs__msg__Paths__rosidl_typesupport_introspection_c__size_function__Paths__paths(
  const void * untyped_member)
{
  const nav_msgs__msg__Path__Sequence * member =
    (const nav_msgs__msg__Path__Sequence *)(untyped_member);
  return member->size;
}

const void * rviz_legged_msgs__msg__Paths__rosidl_typesupport_introspection_c__get_const_function__Paths__paths(
  const void * untyped_member, size_t index)
{
  const nav_msgs__msg__Path__Sequence * member =
    (const nav_msgs__msg__Path__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rviz_legged_msgs__msg__Paths__rosidl_typesupport_introspection_c__get_function__Paths__paths(
  void * untyped_member, size_t index)
{
  nav_msgs__msg__Path__Sequence * member =
    (nav_msgs__msg__Path__Sequence *)(untyped_member);
  return &member->data[index];
}

void rviz_legged_msgs__msg__Paths__rosidl_typesupport_introspection_c__fetch_function__Paths__paths(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const nav_msgs__msg__Path * item =
    ((const nav_msgs__msg__Path *)
    rviz_legged_msgs__msg__Paths__rosidl_typesupport_introspection_c__get_const_function__Paths__paths(untyped_member, index));
  nav_msgs__msg__Path * value =
    (nav_msgs__msg__Path *)(untyped_value);
  *value = *item;
}

void rviz_legged_msgs__msg__Paths__rosidl_typesupport_introspection_c__assign_function__Paths__paths(
  void * untyped_member, size_t index, const void * untyped_value)
{
  nav_msgs__msg__Path * item =
    ((nav_msgs__msg__Path *)
    rviz_legged_msgs__msg__Paths__rosidl_typesupport_introspection_c__get_function__Paths__paths(untyped_member, index));
  const nav_msgs__msg__Path * value =
    (const nav_msgs__msg__Path *)(untyped_value);
  *item = *value;
}

bool rviz_legged_msgs__msg__Paths__rosidl_typesupport_introspection_c__resize_function__Paths__paths(
  void * untyped_member, size_t size)
{
  nav_msgs__msg__Path__Sequence * member =
    (nav_msgs__msg__Path__Sequence *)(untyped_member);
  nav_msgs__msg__Path__Sequence__fini(member);
  return nav_msgs__msg__Path__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember rviz_legged_msgs__msg__Paths__rosidl_typesupport_introspection_c__Paths_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rviz_legged_msgs__msg__Paths, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "paths",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rviz_legged_msgs__msg__Paths, paths),  // bytes offset in struct
    NULL,  // default value
    rviz_legged_msgs__msg__Paths__rosidl_typesupport_introspection_c__size_function__Paths__paths,  // size() function pointer
    rviz_legged_msgs__msg__Paths__rosidl_typesupport_introspection_c__get_const_function__Paths__paths,  // get_const(index) function pointer
    rviz_legged_msgs__msg__Paths__rosidl_typesupport_introspection_c__get_function__Paths__paths,  // get(index) function pointer
    rviz_legged_msgs__msg__Paths__rosidl_typesupport_introspection_c__fetch_function__Paths__paths,  // fetch(index, &value) function pointer
    rviz_legged_msgs__msg__Paths__rosidl_typesupport_introspection_c__assign_function__Paths__paths,  // assign(index, value) function pointer
    rviz_legged_msgs__msg__Paths__rosidl_typesupport_introspection_c__resize_function__Paths__paths  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rviz_legged_msgs__msg__Paths__rosidl_typesupport_introspection_c__Paths_message_members = {
  "rviz_legged_msgs__msg",  // message namespace
  "Paths",  // message name
  2,  // number of fields
  sizeof(rviz_legged_msgs__msg__Paths),
  rviz_legged_msgs__msg__Paths__rosidl_typesupport_introspection_c__Paths_message_member_array,  // message members
  rviz_legged_msgs__msg__Paths__rosidl_typesupport_introspection_c__Paths_init_function,  // function to initialize message memory (memory has to be allocated)
  rviz_legged_msgs__msg__Paths__rosidl_typesupport_introspection_c__Paths_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rviz_legged_msgs__msg__Paths__rosidl_typesupport_introspection_c__Paths_message_type_support_handle = {
  0,
  &rviz_legged_msgs__msg__Paths__rosidl_typesupport_introspection_c__Paths_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rviz_legged_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rviz_legged_msgs, msg, Paths)() {
  rviz_legged_msgs__msg__Paths__rosidl_typesupport_introspection_c__Paths_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  rviz_legged_msgs__msg__Paths__rosidl_typesupport_introspection_c__Paths_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nav_msgs, msg, Path)();
  if (!rviz_legged_msgs__msg__Paths__rosidl_typesupport_introspection_c__Paths_message_type_support_handle.typesupport_identifier) {
    rviz_legged_msgs__msg__Paths__rosidl_typesupport_introspection_c__Paths_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rviz_legged_msgs__msg__Paths__rosidl_typesupport_introspection_c__Paths_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
