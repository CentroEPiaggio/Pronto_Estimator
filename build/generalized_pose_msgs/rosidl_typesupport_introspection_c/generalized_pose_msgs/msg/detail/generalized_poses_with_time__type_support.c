// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from generalized_pose_msgs:msg/GeneralizedPosesWithTime.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "generalized_pose_msgs/msg/detail/generalized_poses_with_time__rosidl_typesupport_introspection_c.h"
#include "generalized_pose_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "generalized_pose_msgs/msg/detail/generalized_poses_with_time__functions.h"
#include "generalized_pose_msgs/msg/detail/generalized_poses_with_time__struct.h"


// Include directives for member types
// Member `generalized_poses_with_time`
#include "generalized_pose_msgs/msg/generalized_pose_with_time.h"
// Member `generalized_poses_with_time`
#include "generalized_pose_msgs/msg/detail/generalized_pose_with_time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void generalized_pose_msgs__msg__GeneralizedPosesWithTime__rosidl_typesupport_introspection_c__GeneralizedPosesWithTime_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  generalized_pose_msgs__msg__GeneralizedPosesWithTime__init(message_memory);
}

void generalized_pose_msgs__msg__GeneralizedPosesWithTime__rosidl_typesupport_introspection_c__GeneralizedPosesWithTime_fini_function(void * message_memory)
{
  generalized_pose_msgs__msg__GeneralizedPosesWithTime__fini(message_memory);
}

size_t generalized_pose_msgs__msg__GeneralizedPosesWithTime__rosidl_typesupport_introspection_c__size_function__GeneralizedPosesWithTime__generalized_poses_with_time(
  const void * untyped_member)
{
  const generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence * member =
    (const generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence *)(untyped_member);
  return member->size;
}

const void * generalized_pose_msgs__msg__GeneralizedPosesWithTime__rosidl_typesupport_introspection_c__get_const_function__GeneralizedPosesWithTime__generalized_poses_with_time(
  const void * untyped_member, size_t index)
{
  const generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence * member =
    (const generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence *)(untyped_member);
  return &member->data[index];
}

void * generalized_pose_msgs__msg__GeneralizedPosesWithTime__rosidl_typesupport_introspection_c__get_function__GeneralizedPosesWithTime__generalized_poses_with_time(
  void * untyped_member, size_t index)
{
  generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence * member =
    (generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence *)(untyped_member);
  return &member->data[index];
}

void generalized_pose_msgs__msg__GeneralizedPosesWithTime__rosidl_typesupport_introspection_c__fetch_function__GeneralizedPosesWithTime__generalized_poses_with_time(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const generalized_pose_msgs__msg__GeneralizedPoseWithTime * item =
    ((const generalized_pose_msgs__msg__GeneralizedPoseWithTime *)
    generalized_pose_msgs__msg__GeneralizedPosesWithTime__rosidl_typesupport_introspection_c__get_const_function__GeneralizedPosesWithTime__generalized_poses_with_time(untyped_member, index));
  generalized_pose_msgs__msg__GeneralizedPoseWithTime * value =
    (generalized_pose_msgs__msg__GeneralizedPoseWithTime *)(untyped_value);
  *value = *item;
}

void generalized_pose_msgs__msg__GeneralizedPosesWithTime__rosidl_typesupport_introspection_c__assign_function__GeneralizedPosesWithTime__generalized_poses_with_time(
  void * untyped_member, size_t index, const void * untyped_value)
{
  generalized_pose_msgs__msg__GeneralizedPoseWithTime * item =
    ((generalized_pose_msgs__msg__GeneralizedPoseWithTime *)
    generalized_pose_msgs__msg__GeneralizedPosesWithTime__rosidl_typesupport_introspection_c__get_function__GeneralizedPosesWithTime__generalized_poses_with_time(untyped_member, index));
  const generalized_pose_msgs__msg__GeneralizedPoseWithTime * value =
    (const generalized_pose_msgs__msg__GeneralizedPoseWithTime *)(untyped_value);
  *item = *value;
}

bool generalized_pose_msgs__msg__GeneralizedPosesWithTime__rosidl_typesupport_introspection_c__resize_function__GeneralizedPosesWithTime__generalized_poses_with_time(
  void * untyped_member, size_t size)
{
  generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence * member =
    (generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence *)(untyped_member);
  generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence__fini(member);
  return generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember generalized_pose_msgs__msg__GeneralizedPosesWithTime__rosidl_typesupport_introspection_c__GeneralizedPosesWithTime_message_member_array[1] = {
  {
    "generalized_poses_with_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(generalized_pose_msgs__msg__GeneralizedPosesWithTime, generalized_poses_with_time),  // bytes offset in struct
    NULL,  // default value
    generalized_pose_msgs__msg__GeneralizedPosesWithTime__rosidl_typesupport_introspection_c__size_function__GeneralizedPosesWithTime__generalized_poses_with_time,  // size() function pointer
    generalized_pose_msgs__msg__GeneralizedPosesWithTime__rosidl_typesupport_introspection_c__get_const_function__GeneralizedPosesWithTime__generalized_poses_with_time,  // get_const(index) function pointer
    generalized_pose_msgs__msg__GeneralizedPosesWithTime__rosidl_typesupport_introspection_c__get_function__GeneralizedPosesWithTime__generalized_poses_with_time,  // get(index) function pointer
    generalized_pose_msgs__msg__GeneralizedPosesWithTime__rosidl_typesupport_introspection_c__fetch_function__GeneralizedPosesWithTime__generalized_poses_with_time,  // fetch(index, &value) function pointer
    generalized_pose_msgs__msg__GeneralizedPosesWithTime__rosidl_typesupport_introspection_c__assign_function__GeneralizedPosesWithTime__generalized_poses_with_time,  // assign(index, value) function pointer
    generalized_pose_msgs__msg__GeneralizedPosesWithTime__rosidl_typesupport_introspection_c__resize_function__GeneralizedPosesWithTime__generalized_poses_with_time  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers generalized_pose_msgs__msg__GeneralizedPosesWithTime__rosidl_typesupport_introspection_c__GeneralizedPosesWithTime_message_members = {
  "generalized_pose_msgs__msg",  // message namespace
  "GeneralizedPosesWithTime",  // message name
  1,  // number of fields
  sizeof(generalized_pose_msgs__msg__GeneralizedPosesWithTime),
  generalized_pose_msgs__msg__GeneralizedPosesWithTime__rosidl_typesupport_introspection_c__GeneralizedPosesWithTime_message_member_array,  // message members
  generalized_pose_msgs__msg__GeneralizedPosesWithTime__rosidl_typesupport_introspection_c__GeneralizedPosesWithTime_init_function,  // function to initialize message memory (memory has to be allocated)
  generalized_pose_msgs__msg__GeneralizedPosesWithTime__rosidl_typesupport_introspection_c__GeneralizedPosesWithTime_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t generalized_pose_msgs__msg__GeneralizedPosesWithTime__rosidl_typesupport_introspection_c__GeneralizedPosesWithTime_message_type_support_handle = {
  0,
  &generalized_pose_msgs__msg__GeneralizedPosesWithTime__rosidl_typesupport_introspection_c__GeneralizedPosesWithTime_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_generalized_pose_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, generalized_pose_msgs, msg, GeneralizedPosesWithTime)() {
  generalized_pose_msgs__msg__GeneralizedPosesWithTime__rosidl_typesupport_introspection_c__GeneralizedPosesWithTime_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, generalized_pose_msgs, msg, GeneralizedPoseWithTime)();
  if (!generalized_pose_msgs__msg__GeneralizedPosesWithTime__rosidl_typesupport_introspection_c__GeneralizedPosesWithTime_message_type_support_handle.typesupport_identifier) {
    generalized_pose_msgs__msg__GeneralizedPosesWithTime__rosidl_typesupport_introspection_c__GeneralizedPosesWithTime_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &generalized_pose_msgs__msg__GeneralizedPosesWithTime__rosidl_typesupport_introspection_c__GeneralizedPosesWithTime_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
