// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from pronto_msgs:msg/FilterState.idl
// generated code does not contain a copyright notice

#ifndef PRONTO_MSGS__MSG__DETAIL__FILTER_STATE__FUNCTIONS_H_
#define PRONTO_MSGS__MSG__DETAIL__FILTER_STATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "pronto_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "pronto_msgs/msg/detail/filter_state__struct.h"

/// Initialize msg/FilterState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * pronto_msgs__msg__FilterState
 * )) before or use
 * pronto_msgs__msg__FilterState__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
bool
pronto_msgs__msg__FilterState__init(pronto_msgs__msg__FilterState * msg);

/// Finalize msg/FilterState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
void
pronto_msgs__msg__FilterState__fini(pronto_msgs__msg__FilterState * msg);

/// Create msg/FilterState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * pronto_msgs__msg__FilterState__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
pronto_msgs__msg__FilterState *
pronto_msgs__msg__FilterState__create();

/// Destroy msg/FilterState message.
/**
 * It calls
 * pronto_msgs__msg__FilterState__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
void
pronto_msgs__msg__FilterState__destroy(pronto_msgs__msg__FilterState * msg);


/// Initialize array of msg/FilterState messages.
/**
 * It allocates the memory for the number of elements and calls
 * pronto_msgs__msg__FilterState__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
bool
pronto_msgs__msg__FilterState__Sequence__init(pronto_msgs__msg__FilterState__Sequence * array, size_t size);

/// Finalize array of msg/FilterState messages.
/**
 * It calls
 * pronto_msgs__msg__FilterState__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
void
pronto_msgs__msg__FilterState__Sequence__fini(pronto_msgs__msg__FilterState__Sequence * array);

/// Create array of msg/FilterState messages.
/**
 * It allocates the memory for the array and calls
 * pronto_msgs__msg__FilterState__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
pronto_msgs__msg__FilterState__Sequence *
pronto_msgs__msg__FilterState__Sequence__create(size_t size);

/// Destroy array of msg/FilterState messages.
/**
 * It calls
 * pronto_msgs__msg__FilterState__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
void
pronto_msgs__msg__FilterState__Sequence__destroy(pronto_msgs__msg__FilterState__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // PRONTO_MSGS__MSG__DETAIL__FILTER_STATE__FUNCTIONS_H_
