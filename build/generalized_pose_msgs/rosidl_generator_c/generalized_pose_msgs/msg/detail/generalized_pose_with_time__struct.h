// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from generalized_pose_msgs:msg/GeneralizedPoseWithTime.idl
// generated code does not contain a copyright notice

#ifndef GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSE_WITH_TIME__STRUCT_H_
#define GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSE_WITH_TIME__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'generalized_pose'
#include "generalized_pose_msgs/msg/detail/generalized_pose__struct.h"

/// Struct defined in msg/GeneralizedPoseWithTime in the package generalized_pose_msgs.
typedef struct generalized_pose_msgs__msg__GeneralizedPoseWithTime
{
  generalized_pose_msgs__msg__GeneralizedPose generalized_pose;
  double time;
} generalized_pose_msgs__msg__GeneralizedPoseWithTime;

// Struct for a sequence of generalized_pose_msgs__msg__GeneralizedPoseWithTime.
typedef struct generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence
{
  generalized_pose_msgs__msg__GeneralizedPoseWithTime * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSE_WITH_TIME__STRUCT_H_
