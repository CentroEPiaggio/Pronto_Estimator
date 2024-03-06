// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from pronto_msgs:msg/JointStateWithAcceleration.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "pronto_msgs/msg/detail/joint_state_with_acceleration__rosidl_typesupport_introspection_c.h"
#include "pronto_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "pronto_msgs/msg/detail/joint_state_with_acceleration__functions.h"
#include "pronto_msgs/msg/detail/joint_state_with_acceleration__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `position`
// Member `velocity`
// Member `acceleration`
// Member `effort`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void JointStateWithAcceleration__rosidl_typesupport_introspection_c__JointStateWithAcceleration_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pronto_msgs__msg__JointStateWithAcceleration__init(message_memory);
}

void JointStateWithAcceleration__rosidl_typesupport_introspection_c__JointStateWithAcceleration_fini_function(void * message_memory)
{
  pronto_msgs__msg__JointStateWithAcceleration__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember JointStateWithAcceleration__rosidl_typesupport_introspection_c__JointStateWithAcceleration_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pronto_msgs__msg__JointStateWithAcceleration, header),  // bytes offset in struct
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
    offsetof(pronto_msgs__msg__JointStateWithAcceleration, name),  // bytes offset in struct
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
    offsetof(pronto_msgs__msg__JointStateWithAcceleration, position),  // bytes offset in struct
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
    offsetof(pronto_msgs__msg__JointStateWithAcceleration, velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "acceleration",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pronto_msgs__msg__JointStateWithAcceleration, acceleration),  // bytes offset in struct
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
    offsetof(pronto_msgs__msg__JointStateWithAcceleration, effort),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers JointStateWithAcceleration__rosidl_typesupport_introspection_c__JointStateWithAcceleration_message_members = {
  "pronto_msgs__msg",  // message namespace
  "JointStateWithAcceleration",  // message name
  6,  // number of fields
  sizeof(pronto_msgs__msg__JointStateWithAcceleration),
  JointStateWithAcceleration__rosidl_typesupport_introspection_c__JointStateWithAcceleration_message_member_array,  // message members
  JointStateWithAcceleration__rosidl_typesupport_introspection_c__JointStateWithAcceleration_init_function,  // function to initialize message memory (memory has to be allocated)
  JointStateWithAcceleration__rosidl_typesupport_introspection_c__JointStateWithAcceleration_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t JointStateWithAcceleration__rosidl_typesupport_introspection_c__JointStateWithAcceleration_message_type_support_handle = {
  0,
  &JointStateWithAcceleration__rosidl_typesupport_introspection_c__JointStateWithAcceleration_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pronto_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pronto_msgs, msg, JointStateWithAcceleration)() {
  JointStateWithAcceleration__rosidl_typesupport_introspection_c__JointStateWithAcceleration_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!JointStateWithAcceleration__rosidl_typesupport_introspection_c__JointStateWithAcceleration_message_type_support_handle.typesupport_identifier) {
    JointStateWithAcceleration__rosidl_typesupport_introspection_c__JointStateWithAcceleration_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &JointStateWithAcceleration__rosidl_typesupport_introspection_c__JointStateWithAcceleration_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
