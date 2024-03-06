// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from generalized_pose_msgs:msg/GeneralizedPoseWithTime.idl
// generated code does not contain a copyright notice

#ifndef GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSE_WITH_TIME__FUNCTIONS_H_
#define GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSE_WITH_TIME__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "generalized_pose_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "generalized_pose_msgs/msg/detail/generalized_pose_with_time__struct.h"

/// Initialize msg/GeneralizedPoseWithTime message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * generalized_pose_msgs__msg__GeneralizedPoseWithTime
 * )) before or use
 * generalized_pose_msgs__msg__GeneralizedPoseWithTime__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_generalized_pose_msgs
bool
generalized_pose_msgs__msg__GeneralizedPoseWithTime__init(generalized_pose_msgs__msg__GeneralizedPoseWithTime * msg);

/// Finalize msg/GeneralizedPoseWithTime message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_generalized_pose_msgs
void
generalized_pose_msgs__msg__GeneralizedPoseWithTime__fini(generalized_pose_msgs__msg__GeneralizedPoseWithTime * msg);

/// Create msg/GeneralizedPoseWithTime message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * generalized_pose_msgs__msg__GeneralizedPoseWithTime__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_generalized_pose_msgs
generalized_pose_msgs__msg__GeneralizedPoseWithTime *
generalized_pose_msgs__msg__GeneralizedPoseWithTime__create();

/// Destroy msg/GeneralizedPoseWithTime message.
/**
 * It calls
 * generalized_pose_msgs__msg__GeneralizedPoseWithTime__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_generalized_pose_msgs
void
generalized_pose_msgs__msg__GeneralizedPoseWithTime__destroy(generalized_pose_msgs__msg__GeneralizedPoseWithTime * msg);

/// Check for msg/GeneralizedPoseWithTime message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_generalized_pose_msgs
bool
generalized_pose_msgs__msg__GeneralizedPoseWithTime__are_equal(const generalized_pose_msgs__msg__GeneralizedPoseWithTime * lhs, const generalized_pose_msgs__msg__GeneralizedPoseWithTime * rhs);

/// Copy a msg/GeneralizedPoseWithTime message.
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
ROSIDL_GENERATOR_C_PUBLIC_generalized_pose_msgs
bool
generalized_pose_msgs__msg__GeneralizedPoseWithTime__copy(
  const generalized_pose_msgs__msg__GeneralizedPoseWithTime * input,
  generalized_pose_msgs__msg__GeneralizedPoseWithTime * output);

/// Initialize array of msg/GeneralizedPoseWithTime messages.
/**
 * It allocates the memory for the number of elements and calls
 * generalized_pose_msgs__msg__GeneralizedPoseWithTime__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_generalized_pose_msgs
bool
generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence__init(generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence * array, size_t size);

/// Finalize array of msg/GeneralizedPoseWithTime messages.
/**
 * It calls
 * generalized_pose_msgs__msg__GeneralizedPoseWithTime__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_generalized_pose_msgs
void
generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence__fini(generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence * array);

/// Create array of msg/GeneralizedPoseWithTime messages.
/**
 * It allocates the memory for the array and calls
 * generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_generalized_pose_msgs
generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence *
generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence__create(size_t size);

/// Destroy array of msg/GeneralizedPoseWithTime messages.
/**
 * It calls
 * generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_generalized_pose_msgs
void
generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence__destroy(generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence * array);

/// Check for msg/GeneralizedPoseWithTime message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_generalized_pose_msgs
bool
generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence__are_equal(const generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence * lhs, const generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence * rhs);

/// Copy an array of msg/GeneralizedPoseWithTime messages.
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
ROSIDL_GENERATOR_C_PUBLIC_generalized_pose_msgs
bool
generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence__copy(
  const generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence * input,
  generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSE_WITH_TIME__FUNCTIONS_H_
