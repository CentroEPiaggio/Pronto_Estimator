// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from pronto_msgs:msg/ControllerFootContact.idl
// generated code does not contain a copyright notice

#ifndef PRONTO_MSGS__MSG__DETAIL__CONTROLLER_FOOT_CONTACT__FUNCTIONS_H_
#define PRONTO_MSGS__MSG__DETAIL__CONTROLLER_FOOT_CONTACT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "pronto_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "pronto_msgs/msg/detail/controller_foot_contact__struct.h"

/// Initialize msg/ControllerFootContact message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * pronto_msgs__msg__ControllerFootContact
 * )) before or use
 * pronto_msgs__msg__ControllerFootContact__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
bool
pronto_msgs__msg__ControllerFootContact__init(pronto_msgs__msg__ControllerFootContact * msg);

/// Finalize msg/ControllerFootContact message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
void
pronto_msgs__msg__ControllerFootContact__fini(pronto_msgs__msg__ControllerFootContact * msg);

/// Create msg/ControllerFootContact message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * pronto_msgs__msg__ControllerFootContact__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
pronto_msgs__msg__ControllerFootContact *
pronto_msgs__msg__ControllerFootContact__create();

/// Destroy msg/ControllerFootContact message.
/**
 * It calls
 * pronto_msgs__msg__ControllerFootContact__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
void
pronto_msgs__msg__ControllerFootContact__destroy(pronto_msgs__msg__ControllerFootContact * msg);


/// Initialize array of msg/ControllerFootContact messages.
/**
 * It allocates the memory for the number of elements and calls
 * pronto_msgs__msg__ControllerFootContact__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
bool
pronto_msgs__msg__ControllerFootContact__Sequence__init(pronto_msgs__msg__ControllerFootContact__Sequence * array, size_t size);

/// Finalize array of msg/ControllerFootContact messages.
/**
 * It calls
 * pronto_msgs__msg__ControllerFootContact__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
void
pronto_msgs__msg__ControllerFootContact__Sequence__fini(pronto_msgs__msg__ControllerFootContact__Sequence * array);

/// Create array of msg/ControllerFootContact messages.
/**
 * It allocates the memory for the array and calls
 * pronto_msgs__msg__ControllerFootContact__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
pronto_msgs__msg__ControllerFootContact__Sequence *
pronto_msgs__msg__ControllerFootContact__Sequence__create(size_t size);

/// Destroy array of msg/ControllerFootContact messages.
/**
 * It calls
 * pronto_msgs__msg__ControllerFootContact__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pronto_msgs
void
pronto_msgs__msg__ControllerFootContact__Sequence__destroy(pronto_msgs__msg__ControllerFootContact__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // PRONTO_MSGS__MSG__DETAIL__CONTROLLER_FOOT_CONTACT__FUNCTIONS_H_