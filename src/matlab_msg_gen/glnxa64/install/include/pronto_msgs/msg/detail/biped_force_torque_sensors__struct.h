// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pronto_msgs:msg/BipedForceTorqueSensors.idl
// generated code does not contain a copyright notice

#ifndef PRONTO_MSGS__MSG__DETAIL__BIPED_FORCE_TORQUE_SENSORS__STRUCT_H_
#define PRONTO_MSGS__MSG__DETAIL__BIPED_FORCE_TORQUE_SENSORS__STRUCT_H_

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
// Member 'l_foot'
// Member 'r_foot'
// Member 'l_hand'
// Member 'r_hand'
#include "geometry_msgs/msg/detail/wrench__struct.h"

// Struct defined in msg/BipedForceTorqueSensors in the package pronto_msgs.
typedef struct pronto_msgs__msg__BipedForceTorqueSensors
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Wrench l_foot;
  geometry_msgs__msg__Wrench r_foot;
  geometry_msgs__msg__Wrench l_hand;
  geometry_msgs__msg__Wrench r_hand;
} pronto_msgs__msg__BipedForceTorqueSensors;

// Struct for a sequence of pronto_msgs__msg__BipedForceTorqueSensors.
typedef struct pronto_msgs__msg__BipedForceTorqueSensors__Sequence
{
  pronto_msgs__msg__BipedForceTorqueSensors * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pronto_msgs__msg__BipedForceTorqueSensors__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PRONTO_MSGS__MSG__DETAIL__BIPED_FORCE_TORQUE_SENSORS__STRUCT_H_
