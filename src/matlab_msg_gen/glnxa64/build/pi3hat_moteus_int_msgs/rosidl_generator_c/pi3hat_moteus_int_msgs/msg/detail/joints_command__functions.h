// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from pi3hat_moteus_int_msgs:msg/JointsCommand.idl
// generated code does not contain a copyright notice

#ifndef PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__JOINTS_COMMAND__FUNCTIONS_H_
#define PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__JOINTS_COMMAND__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "pi3hat_moteus_int_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "pi3hat_moteus_int_msgs/msg/detail/joints_command__struct.h"

/// Initialize msg/JointsCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * pi3hat_moteus_int_msgs__msg__JointsCommand
 * )) before or use
 * pi3hat_moteus_int_msgs__msg__JointsCommand__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_pi3hat_moteus_int_msgs
bool
pi3hat_moteus_int_msgs__msg__JointsCommand__init(pi3hat_moteus_int_msgs__msg__JointsCommand * msg);

/// Finalize msg/JointsCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pi3hat_moteus_int_msgs
void
pi3hat_moteus_int_msgs__msg__JointsCommand__fini(pi3hat_moteus_int_msgs__msg__JointsCommand * msg);

/// Create msg/JointsCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * pi3hat_moteus_int_msgs__msg__JointsCommand__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pi3hat_moteus_int_msgs
pi3hat_moteus_int_msgs__msg__JointsCommand *
pi3hat_moteus_int_msgs__msg__JointsCommand__create();

/// Destroy msg/JointsCommand message.
/**
 * It calls
 * pi3hat_moteus_int_msgs__msg__JointsCommand__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pi3hat_moteus_int_msgs
void
pi3hat_moteus_int_msgs__msg__JointsCommand__destroy(pi3hat_moteus_int_msgs__msg__JointsCommand * msg);


/// Initialize array of msg/JointsCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * pi3hat_moteus_int_msgs__msg__JointsCommand__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_pi3hat_moteus_int_msgs
bool
pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence__init(pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence * array, size_t size);

/// Finalize array of msg/JointsCommand messages.
/**
 * It calls
 * pi3hat_moteus_int_msgs__msg__JointsCommand__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pi3hat_moteus_int_msgs
void
pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence__fini(pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence * array);

/// Create array of msg/JointsCommand messages.
/**
 * It allocates the memory for the array and calls
 * pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pi3hat_moteus_int_msgs
pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence *
pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence__create(size_t size);

/// Destroy array of msg/JointsCommand messages.
/**
 * It calls
 * pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pi3hat_moteus_int_msgs
void
pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence__destroy(pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__JOINTS_COMMAND__FUNCTIONS_H_
