// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from generalized_pose_msgs:msg/GeneralizedPose.idl
// generated code does not contain a copyright notice

#ifndef GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSE__STRUCT_H_
#define GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'base_acc'
// Member 'base_vel'
// Member 'base_pos'
// Member 'base_angvel'
#include "geometry_msgs/msg/detail/vector3__struct.h"
// Member 'base_quat'
#include "geometry_msgs/msg/detail/quaternion__struct.h"
// Member 'feet_acc'
// Member 'feet_vel'
// Member 'feet_pos'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'contact_feet'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/GeneralizedPose in the package generalized_pose_msgs.
/**
  * A representation of all the quantities that define the desired pose of the robot base and of the swing feet, called desired generalized pose.
 */
typedef struct generalized_pose_msgs__msg__GeneralizedPose
{
  /// Base linear quantities
  geometry_msgs__msg__Vector3 base_acc;
  geometry_msgs__msg__Vector3 base_vel;
  geometry_msgs__msg__Vector3 base_pos;
  /// Base angular quantities
  geometry_msgs__msg__Vector3 base_angvel;
  geometry_msgs__msg__Quaternion base_quat;
  /// Swing feet linear quantities
  rosidl_runtime_c__double__Sequence feet_acc;
  rosidl_runtime_c__double__Sequence feet_vel;
  rosidl_runtime_c__double__Sequence feet_pos;
  /// List of feet names in contact with the ground
  rosidl_runtime_c__String__Sequence contact_feet;
} generalized_pose_msgs__msg__GeneralizedPose;

// Struct for a sequence of generalized_pose_msgs__msg__GeneralizedPose.
typedef struct generalized_pose_msgs__msg__GeneralizedPose__Sequence
{
  generalized_pose_msgs__msg__GeneralizedPose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} generalized_pose_msgs__msg__GeneralizedPose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSE__STRUCT_H_
