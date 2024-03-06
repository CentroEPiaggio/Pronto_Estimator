// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from velocity_command_msgs:msg/SimpleVelocityCommand.idl
// generated code does not contain a copyright notice

#ifndef VELOCITY_COMMAND_MSGS__MSG__DETAIL__SIMPLE_VELOCITY_COMMAND__STRUCT_H_
#define VELOCITY_COMMAND_MSGS__MSG__DETAIL__SIMPLE_VELOCITY_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/SimpleVelocityCommand in the package velocity_command_msgs.
/**
  * The simplest velocity command given to a robot: forward velocity, lateral velocity, and yaw rate.
 */
typedef struct velocity_command_msgs__msg__SimpleVelocityCommand
{
  double velocity_forward;
  double velocity_lateral;
  double yaw_rate;
} velocity_command_msgs__msg__SimpleVelocityCommand;

// Struct for a sequence of velocity_command_msgs__msg__SimpleVelocityCommand.
typedef struct velocity_command_msgs__msg__SimpleVelocityCommand__Sequence
{
  velocity_command_msgs__msg__SimpleVelocityCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} velocity_command_msgs__msg__SimpleVelocityCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VELOCITY_COMMAND_MSGS__MSG__DETAIL__SIMPLE_VELOCITY_COMMAND__STRUCT_H_
