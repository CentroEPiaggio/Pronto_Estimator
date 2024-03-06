// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from velocity_command_msgs:msg/SimpleVelocityCommand.idl
// generated code does not contain a copyright notice

#ifndef VELOCITY_COMMAND_MSGS__MSG__DETAIL__SIMPLE_VELOCITY_COMMAND__FUNCTIONS_H_
#define VELOCITY_COMMAND_MSGS__MSG__DETAIL__SIMPLE_VELOCITY_COMMAND__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "velocity_command_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "velocity_command_msgs/msg/detail/simple_velocity_command__struct.h"

/// Initialize msg/SimpleVelocityCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * velocity_command_msgs__msg__SimpleVelocityCommand
 * )) before or use
 * velocity_command_msgs__msg__SimpleVelocityCommand__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_velocity_command_msgs
bool
velocity_command_msgs__msg__SimpleVelocityCommand__init(velocity_command_msgs__msg__SimpleVelocityCommand * msg);

/// Finalize msg/SimpleVelocityCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_velocity_command_msgs
void
velocity_command_msgs__msg__SimpleVelocityCommand__fini(velocity_command_msgs__msg__SimpleVelocityCommand * msg);

/// Create msg/SimpleVelocityCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * velocity_command_msgs__msg__SimpleVelocityCommand__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_velocity_command_msgs
velocity_command_msgs__msg__SimpleVelocityCommand *
velocity_command_msgs__msg__SimpleVelocityCommand__create();

/// Destroy msg/SimpleVelocityCommand message.
/**
 * It calls
 * velocity_command_msgs__msg__SimpleVelocityCommand__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_velocity_command_msgs
void
velocity_command_msgs__msg__SimpleVelocityCommand__destroy(velocity_command_msgs__msg__SimpleVelocityCommand * msg);

/// Check for msg/SimpleVelocityCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_velocity_command_msgs
bool
velocity_command_msgs__msg__SimpleVelocityCommand__are_equal(const velocity_command_msgs__msg__SimpleVelocityCommand * lhs, const velocity_command_msgs__msg__SimpleVelocityCommand * rhs);

/// Copy a msg/SimpleVelocityCommand message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_velocity_command_msgs
bool
velocity_command_msgs__msg__SimpleVelocityCommand__copy(
  const velocity_command_msgs__msg__SimpleVelocityCommand * input,
  velocity_command_msgs__msg__SimpleVelocityCommand * output);

/// Initialize array of msg/SimpleVelocityCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * velocity_command_msgs__msg__SimpleVelocityCommand__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_velocity_command_msgs
bool
velocity_command_msgs__msg__SimpleVelocityCommand__Sequence__init(velocity_command_msgs__msg__SimpleVelocityCommand__Sequence * array, size_t size);

/// Finalize array of msg/SimpleVelocityCommand messages.
/**
 * It calls
 * velocity_command_msgs__msg__SimpleVelocityCommand__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_velocity_command_msgs
void
velocity_command_msgs__msg__SimpleVelocityCommand__Sequence__fini(velocity_command_msgs__msg__SimpleVelocityCommand__Sequence * array);

/// Create array of msg/SimpleVelocityCommand messages.
/**
 * It allocates the memory for the array and calls
 * velocity_command_msgs__msg__SimpleVelocityCommand__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_velocity_command_msgs
velocity_command_msgs__msg__SimpleVelocityCommand__Sequence *
velocity_command_msgs__msg__SimpleVelocityCommand__Sequence__create(size_t size);

/// Destroy array of msg/SimpleVelocityCommand messages.
/**
 * It calls
 * velocity_command_msgs__msg__SimpleVelocityCommand__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_velocity_command_msgs
void
velocity_command_msgs__msg__SimpleVelocityCommand__Sequence__destroy(velocity_command_msgs__msg__SimpleVelocityCommand__Sequence * array);

/// Check for msg/SimpleVelocityCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_velocity_command_msgs
bool
velocity_command_msgs__msg__SimpleVelocityCommand__Sequence__are_equal(const velocity_command_msgs__msg__SimpleVelocityCommand__Sequence * lhs, const velocity_command_msgs__msg__SimpleVelocityCommand__Sequence * rhs);

/// Copy an array of msg/SimpleVelocityCommand messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_velocity_command_msgs
bool
velocity_command_msgs__msg__SimpleVelocityCommand__Sequence__copy(
  const velocity_command_msgs__msg__SimpleVelocityCommand__Sequence * input,
  velocity_command_msgs__msg__SimpleVelocityCommand__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // VELOCITY_COMMAND_MSGS__MSG__DETAIL__SIMPLE_VELOCITY_COMMAND__FUNCTIONS_H_
