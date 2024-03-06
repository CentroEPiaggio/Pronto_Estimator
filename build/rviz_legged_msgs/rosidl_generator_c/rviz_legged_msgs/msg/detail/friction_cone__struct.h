// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rviz_legged_msgs:msg/FrictionCone.idl
// generated code does not contain a copyright notice

#ifndef RVIZ_LEGGED_MSGS__MSG__DETAIL__FRICTION_CONE__STRUCT_H_
#define RVIZ_LEGGED_MSGS__MSG__DETAIL__FRICTION_CONE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'normal_direction'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/FrictionCone in the package rviz_legged_msgs.
typedef struct rviz_legged_msgs__msg__FrictionCone
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Vector3 normal_direction;
  double friction_coefficient;
} rviz_legged_msgs__msg__FrictionCone;

// Struct for a sequence of rviz_legged_msgs__msg__FrictionCone.
typedef struct rviz_legged_msgs__msg__FrictionCone__Sequence
{
  rviz_legged_msgs__msg__FrictionCone * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rviz_legged_msgs__msg__FrictionCone__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RVIZ_LEGGED_MSGS__MSG__DETAIL__FRICTION_CONE__STRUCT_H_
