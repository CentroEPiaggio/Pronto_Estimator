// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from generalized_pose_msgs:msg/GeneralizedPosesWithTime.idl
// generated code does not contain a copyright notice

#ifndef GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSES_WITH_TIME__STRUCT_H_
#define GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSES_WITH_TIME__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'generalized_poses_with_time'
#include "generalized_pose_msgs/msg/detail/generalized_pose_with_time__struct.h"

/// Struct defined in msg/GeneralizedPosesWithTime in the package generalized_pose_msgs.
typedef struct generalized_pose_msgs__msg__GeneralizedPosesWithTime
{
  generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence generalized_poses_with_time;
} generalized_pose_msgs__msg__GeneralizedPosesWithTime;

// Struct for a sequence of generalized_pose_msgs__msg__GeneralizedPosesWithTime.
typedef struct generalized_pose_msgs__msg__GeneralizedPosesWithTime__Sequence
{
  generalized_pose_msgs__msg__GeneralizedPosesWithTime * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} generalized_pose_msgs__msg__GeneralizedPosesWithTime__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSES_WITH_TIME__STRUCT_H_
