// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from pronto_msgs:msg/VisualOdometryUpdate.idl
// generated code does not contain a copyright notice

#ifndef PRONTO_MSGS__MSG__DETAIL__VISUAL_ODOMETRY_UPDATE__FUNCTIONS_H_
#define PRONTO_MSGS__MSG__DETAIL__VISUAL_ODOMETRY_UPDATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "pronto_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "pronto_msgs/msg/detail/visual_odometry_update__struct.h"

/// Initialize msg/VisualOdometryUpdate message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * pronto_msgs__msg__VisualOdometryUpdate
 * )) before or use
 * pronto_msgs__msg__VisualOdometryUpdate__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
bool
pronto_msgs__msg__VisualOdometryUpdate__init(pronto_msgs__msg__VisualOdometryUpdate * msg);

/// Finalize msg/VisualOdometryUpdate message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
void
pronto_msgs__msg__VisualOdometryUpdate__fini(pronto_msgs__msg__VisualOdometryUpdate * msg);

/// Create msg/VisualOdometryUpdate message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * pronto_msgs__msg__VisualOdometryUpdate__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
pronto_msgs__msg__VisualOdometryUpdate *
pronto_msgs__msg__VisualOdometryUpdate__create();

/// Destroy msg/VisualOdometryUpdate message.
/**
 * It calls
 * pronto_msgs__msg__VisualOdometryUpdate__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
void
pronto_msgs__msg__VisualOdometryUpdate__destroy(pronto_msgs__msg__VisualOdometryUpdate * msg);


/// Initialize array of msg/VisualOdometryUpdate messages.
/**
 * It allocates the memory for the number of elements and calls
 * pronto_msgs__msg__VisualOdometryUpdate__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
bool
pronto_msgs__msg__VisualOdometryUpdate__Sequence__init(pronto_msgs__msg__VisualOdometryUpdate__Sequence * array, size_t size);

/// Finalize array of msg/VisualOdometryUpdate messages.
/**
 * It calls
 * pronto_msgs__msg__VisualOdometryUpdate__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
void
pronto_msgs__msg__VisualOdometryUpdate__Sequence__fini(pronto_msgs__msg__VisualOdometryUpdate__Sequence * array);

/// Create array of msg/VisualOdometryUpdate messages.
/**
 * It allocates the memory for the array and calls
 * pronto_msgs__msg__VisualOdometryUpdate__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
pronto_msgs__msg__VisualOdometryUpdate__Sequence *
pronto_msgs__msg__VisualOdometryUpdate__Sequence__create(size_t size);

/// Destroy array of msg/VisualOdometryUpdate messages.
/**
 * It calls
 * pronto_msgs__msg__VisualOdometryUpdate__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
void
pronto_msgs__msg__VisualOdometryUpdate__Sequence__destroy(pronto_msgs__msg__VisualOdometryUpdate__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // PRONTO_MSGS__MSG__DETAIL__VISUAL_ODOMETRY_UPDATE__FUNCTIONS_H_
