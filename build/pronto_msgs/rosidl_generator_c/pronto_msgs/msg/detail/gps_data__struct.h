// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pronto_msgs:msg/GPSData.idl
// generated code does not contain a copyright notice

#ifndef PRONTO_MSGS__MSG__DETAIL__GPS_DATA__STRUCT_H_
#define PRONTO_MSGS__MSG__DETAIL__GPS_DATA__STRUCT_H_

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

/// Struct defined in msg/GPSData in the package pronto_msgs.
typedef struct pronto_msgs__msg__GPSData
{
  std_msgs__msg__Header header;
  uint64_t utime;
  int32_t gps_lock;
  double longitude;
  double latitude;
  double elev;
  double horizontal_accuracy;
  double vertical_accuracy;
  uint32_t num_satellites;
  double speed;
  double heading;
  double xyz_pos[3];
  double gps_time;
} pronto_msgs__msg__GPSData;

// Struct for a sequence of pronto_msgs__msg__GPSData.
typedef struct pronto_msgs__msg__GPSData__Sequence
{
  pronto_msgs__msg__GPSData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pronto_msgs__msg__GPSData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PRONTO_MSGS__MSG__DETAIL__GPS_DATA__STRUCT_H_
