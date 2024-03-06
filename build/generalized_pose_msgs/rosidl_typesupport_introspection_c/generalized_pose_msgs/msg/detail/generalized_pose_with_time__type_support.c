// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from generalized_pose_msgs:msg/GeneralizedPoseWithTime.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "generalized_pose_msgs/msg/detail/generalized_pose_with_time__rosidl_typesupport_introspection_c.h"
#include "generalized_pose_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "generalized_pose_msgs/msg/detail/generalized_pose_with_time__functions.h"
#include "generalized_pose_msgs/msg/detail/generalized_pose_with_time__struct.h"


// Include directives for member types
// Member `generalized_pose`
#include "generalized_pose_msgs/msg/generalized_pose.h"
// Member `generalized_pose`
#include "generalized_pose_msgs/msg/detail/generalized_pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void generalized_pose_msgs__msg__GeneralizedPoseWithTime__rosidl_typesupport_introspection_c__GeneralizedPoseWithTime_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  generalized_pose_msgs__msg__GeneralizedPoseWithTime__init(message_memory);
}

void generalized_pose_msgs__msg__GeneralizedPoseWithTime__rosidl_typesupport_introspection_c__GeneralizedPoseWithTime_fini_function(void * message_memory)
{
  generalized_pose_msgs__msg__GeneralizedPoseWithTime__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember generalized_pose_msgs__msg__GeneralizedPoseWithTime__rosidl_typesupport_introspection_c__GeneralizedPoseWithTime_message_member_array[2] = {
  {
    "generalized_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(generalized_pose_msgs__msg__GeneralizedPoseWithTime, generalized_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(generalized_pose_msgs__msg__GeneralizedPoseWithTime, time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers generalized_pose_msgs__msg__GeneralizedPoseWithTime__rosidl_typesupport_introspection_c__GeneralizedPoseWithTime_message_members = {
  "generalized_pose_msgs__msg",  // message namespace
  "GeneralizedPoseWithTime",  // message name
  2,  // number of fields
  sizeof(generalized_pose_msgs__msg__GeneralizedPoseWithTime),
  generalized_pose_msgs__msg__GeneralizedPoseWithTime__rosidl_typesupport_introspection_c__GeneralizedPoseWithTime_message_member_array,  // message members
  generalized_pose_msgs__msg__GeneralizedPoseWithTime__rosidl_typesupport_introspection_c__GeneralizedPoseWithTime_init_function,  // function to initialize message memory (memory has to be allocated)
  generalized_pose_msgs__msg__GeneralizedPoseWithTime__rosidl_typesupport_introspection_c__GeneralizedPoseWithTime_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t generalized_pose_msgs__msg__GeneralizedPoseWithTime__rosidl_typesupport_introspection_c__GeneralizedPoseWithTime_message_type_support_handle = {
  0,
  &generalized_pose_msgs__msg__GeneralizedPoseWithTime__rosidl_typesupport_introspection_c__GeneralizedPoseWithTime_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_generalized_pose_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, generalized_pose_msgs, msg, GeneralizedPoseWithTime)() {
  generalized_pose_msgs__msg__GeneralizedPoseWithTime__rosidl_typesupport_introspection_c__GeneralizedPoseWithTime_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, generalized_pose_msgs, msg, GeneralizedPose)();
  if (!generalized_pose_msgs__msg__GeneralizedPoseWithTime__rosidl_typesupport_introspection_c__GeneralizedPoseWithTime_message_type_support_handle.typesupport_identifier) {
    generalized_pose_msgs__msg__GeneralizedPoseWithTime__rosidl_typesupport_introspection_c__GeneralizedPoseWithTime_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &generalized_pose_msgs__msg__GeneralizedPoseWithTime__rosidl_typesupport_introspection_c__GeneralizedPoseWithTime_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
