// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rviz_legged_msgs:msg/WrenchesStamped.idl
// generated code does not contain a copyright notice

#ifndef RVIZ_LEGGED_MSGS__MSG__DETAIL__WRENCHES_STAMPED__STRUCT_H_
#define RVIZ_LEGGED_MSGS__MSG__DETAIL__WRENCHES_STAMPED__STRUCT_H_

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
// Member 'wrenches_stamped'
#include "geometry_msgs/msg/detail/wrench_stamped__struct.h"

/// Struct defined in msg/WrenchesStamped in the package rviz_legged_msgs.
/**
  * A list of WrenchStamped messages
 */
typedef struct rviz_legged_msgs__msg__WrenchesStamped
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__WrenchStamped__Sequence wrenches_stamped;
} rviz_legged_msgs__msg__WrenchesStamped;

// Struct for a sequence of rviz_legged_msgs__msg__WrenchesStamped.
typedef struct rviz_legged_msgs__msg__WrenchesStamped__Sequence
{
  rviz_legged_msgs__msg__WrenchesStamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rviz_legged_msgs__msg__WrenchesStamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RVIZ_LEGGED_MSGS__MSG__DETAIL__WRENCHES_STAMPED__STRUCT_H_
