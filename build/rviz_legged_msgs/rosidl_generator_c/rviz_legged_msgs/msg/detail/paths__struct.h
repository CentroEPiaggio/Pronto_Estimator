// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rviz_legged_msgs:msg/Paths.idl
// generated code does not contain a copyright notice

#ifndef RVIZ_LEGGED_MSGS__MSG__DETAIL__PATHS__STRUCT_H_
#define RVIZ_LEGGED_MSGS__MSG__DETAIL__PATHS__STRUCT_H_

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
// Member 'paths'
#include "nav_msgs/msg/detail/path__struct.h"

/// Struct defined in msg/Paths in the package rviz_legged_msgs.
typedef struct rviz_legged_msgs__msg__Paths
{
  std_msgs__msg__Header header;
  nav_msgs__msg__Path__Sequence paths;
} rviz_legged_msgs__msg__Paths;

// Struct for a sequence of rviz_legged_msgs__msg__Paths.
typedef struct rviz_legged_msgs__msg__Paths__Sequence
{
  rviz_legged_msgs__msg__Paths * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rviz_legged_msgs__msg__Paths__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RVIZ_LEGGED_MSGS__MSG__DETAIL__PATHS__STRUCT_H_
