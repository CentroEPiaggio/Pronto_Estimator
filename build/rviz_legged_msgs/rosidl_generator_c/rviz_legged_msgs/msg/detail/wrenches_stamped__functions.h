// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rviz_legged_msgs:msg/WrenchesStamped.idl
// generated code does not contain a copyright notice

#ifndef RVIZ_LEGGED_MSGS__MSG__DETAIL__WRENCHES_STAMPED__FUNCTIONS_H_
#define RVIZ_LEGGED_MSGS__MSG__DETAIL__WRENCHES_STAMPED__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "rviz_legged_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "rviz_legged_msgs/msg/detail/wrenches_stamped__struct.h"

/// Initialize msg/WrenchesStamped message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rviz_legged_msgs__msg__WrenchesStamped
 * )) before or use
 * rviz_legged_msgs__msg__WrenchesStamped__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rviz_legged_msgs
bool
rviz_legged_msgs__msg__WrenchesStamped__init(rviz_legged_msgs__msg__WrenchesStamped * msg);

/// Finalize msg/WrenchesStamped message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rviz_legged_msgs
void
rviz_legged_msgs__msg__WrenchesStamped__fini(rviz_legged_msgs__msg__WrenchesStamped * msg);

/// Create msg/WrenchesStamped message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rviz_legged_msgs__msg__WrenchesStamped__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rviz_legged_msgs
rviz_legged_msgs__msg__WrenchesStamped *
rviz_legged_msgs__msg__WrenchesStamped__create();

/// Destroy msg/WrenchesStamped message.
/**
 * It calls
 * rviz_legged_msgs__msg__WrenchesStamped__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rviz_legged_msgs
void
rviz_legged_msgs__msg__WrenchesStamped__destroy(rviz_legged_msgs__msg__WrenchesStamped * msg);

/// Check for msg/WrenchesStamped message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rviz_legged_msgs
bool
rviz_legged_msgs__msg__WrenchesStamped__are_equal(const rviz_legged_msgs__msg__WrenchesStamped * lhs, const rviz_legged_msgs__msg__WrenchesStamped * rhs);

/// Copy a msg/WrenchesStamped message.
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
ROSIDL_GENERATOR_C_PUBLIC_rviz_legged_msgs
bool
rviz_legged_msgs__msg__WrenchesStamped__copy(
  const rviz_legged_msgs__msg__WrenchesStamped * input,
  rviz_legged_msgs__msg__WrenchesStamped * output);

/// Initialize array of msg/WrenchesStamped messages.
/**
 * It allocates the memory for the number of elements and calls
 * rviz_legged_msgs__msg__WrenchesStamped__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rviz_legged_msgs
bool
rviz_legged_msgs__msg__WrenchesStamped__Sequence__init(rviz_legged_msgs__msg__WrenchesStamped__Sequence * array, size_t size);

/// Finalize array of msg/WrenchesStamped messages.
/**
 * It calls
 * rviz_legged_msgs__msg__WrenchesStamped__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rviz_legged_msgs
void
rviz_legged_msgs__msg__WrenchesStamped__Sequence__fini(rviz_legged_msgs__msg__WrenchesStamped__Sequence * array);

/// Create array of msg/WrenchesStamped messages.
/**
 * It allocates the memory for the array and calls
 * rviz_legged_msgs__msg__WrenchesStamped__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rviz_legged_msgs
rviz_legged_msgs__msg__WrenchesStamped__Sequence *
rviz_legged_msgs__msg__WrenchesStamped__Sequence__create(size_t size);

/// Destroy array of msg/WrenchesStamped messages.
/**
 * It calls
 * rviz_legged_msgs__msg__WrenchesStamped__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rviz_legged_msgs
void
rviz_legged_msgs__msg__WrenchesStamped__Sequence__destroy(rviz_legged_msgs__msg__WrenchesStamped__Sequence * array);

/// Check for msg/WrenchesStamped message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rviz_legged_msgs
bool
rviz_legged_msgs__msg__WrenchesStamped__Sequence__are_equal(const rviz_legged_msgs__msg__WrenchesStamped__Sequence * lhs, const rviz_legged_msgs__msg__WrenchesStamped__Sequence * rhs);

/// Copy an array of msg/WrenchesStamped messages.
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
ROSIDL_GENERATOR_C_PUBLIC_rviz_legged_msgs
bool
rviz_legged_msgs__msg__WrenchesStamped__Sequence__copy(
  const rviz_legged_msgs__msg__WrenchesStamped__Sequence * input,
  rviz_legged_msgs__msg__WrenchesStamped__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // RVIZ_LEGGED_MSGS__MSG__DETAIL__WRENCHES_STAMPED__FUNCTIONS_H_
