// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from pronto_msgs:msg/IndexedMeasurement.idl
// generated code does not contain a copyright notice

#ifndef PRONTO_MSGS__MSG__DETAIL__INDEXED_MEASUREMENT__FUNCTIONS_H_
#define PRONTO_MSGS__MSG__DETAIL__INDEXED_MEASUREMENT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "pronto_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "pronto_msgs/msg/detail/indexed_measurement__struct.h"

/// Initialize msg/IndexedMeasurement message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * pronto_msgs__msg__IndexedMeasurement
 * )) before or use
 * pronto_msgs__msg__IndexedMeasurement__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
bool
pronto_msgs__msg__IndexedMeasurement__init(pronto_msgs__msg__IndexedMeasurement * msg);

/// Finalize msg/IndexedMeasurement message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
void
pronto_msgs__msg__IndexedMeasurement__fini(pronto_msgs__msg__IndexedMeasurement * msg);

/// Create msg/IndexedMeasurement message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * pronto_msgs__msg__IndexedMeasurement__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
pronto_msgs__msg__IndexedMeasurement *
pronto_msgs__msg__IndexedMeasurement__create();

/// Destroy msg/IndexedMeasurement message.
/**
 * It calls
 * pronto_msgs__msg__IndexedMeasurement__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
void
pronto_msgs__msg__IndexedMeasurement__destroy(pronto_msgs__msg__IndexedMeasurement * msg);

/// Check for msg/IndexedMeasurement message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
bool
pronto_msgs__msg__IndexedMeasurement__are_equal(const pronto_msgs__msg__IndexedMeasurement * lhs, const pronto_msgs__msg__IndexedMeasurement * rhs);

/// Copy a msg/IndexedMeasurement message.
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
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
bool
pronto_msgs__msg__IndexedMeasurement__copy(
  const pronto_msgs__msg__IndexedMeasurement * input,
  pronto_msgs__msg__IndexedMeasurement * output);

/// Initialize array of msg/IndexedMeasurement messages.
/**
 * It allocates the memory for the number of elements and calls
 * pronto_msgs__msg__IndexedMeasurement__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
bool
pronto_msgs__msg__IndexedMeasurement__Sequence__init(pronto_msgs__msg__IndexedMeasurement__Sequence * array, size_t size);

/// Finalize array of msg/IndexedMeasurement messages.
/**
 * It calls
 * pronto_msgs__msg__IndexedMeasurement__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
void
pronto_msgs__msg__IndexedMeasurement__Sequence__fini(pronto_msgs__msg__IndexedMeasurement__Sequence * array);

/// Create array of msg/IndexedMeasurement messages.
/**
 * It allocates the memory for the array and calls
 * pronto_msgs__msg__IndexedMeasurement__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
pronto_msgs__msg__IndexedMeasurement__Sequence *
pronto_msgs__msg__IndexedMeasurement__Sequence__create(size_t size);

/// Destroy array of msg/IndexedMeasurement messages.
/**
 * It calls
 * pronto_msgs__msg__IndexedMeasurement__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
void
pronto_msgs__msg__IndexedMeasurement__Sequence__destroy(pronto_msgs__msg__IndexedMeasurement__Sequence * array);

/// Check for msg/IndexedMeasurement message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
bool
pronto_msgs__msg__IndexedMeasurement__Sequence__are_equal(const pronto_msgs__msg__IndexedMeasurement__Sequence * lhs, const pronto_msgs__msg__IndexedMeasurement__Sequence * rhs);

/// Copy an array of msg/IndexedMeasurement messages.
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
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
bool
pronto_msgs__msg__IndexedMeasurement__Sequence__copy(
  const pronto_msgs__msg__IndexedMeasurement__Sequence * input,
  pronto_msgs__msg__IndexedMeasurement__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // PRONTO_MSGS__MSG__DETAIL__INDEXED_MEASUREMENT__FUNCTIONS_H_
