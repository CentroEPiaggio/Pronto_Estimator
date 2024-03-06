// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from pronto_msgs:msg/QuadrupedForceTorqueSensors.idl
// generated code does not contain a copyright notice
#include "pronto_msgs/msg/detail/quadruped_force_torque_sensors__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "pronto_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "pronto_msgs/msg/detail/quadruped_force_torque_sensors__struct.h"
#include "pronto_msgs/msg/detail/quadruped_force_torque_sensors__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "geometry_msgs/msg/detail/wrench__functions.h"  // lf, lh, rf, rh
#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pronto_msgs
size_t get_serialized_size_geometry_msgs__msg__Wrench(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pronto_msgs
size_t max_serialized_size_geometry_msgs__msg__Wrench(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pronto_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Wrench)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pronto_msgs
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pronto_msgs
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pronto_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _QuadrupedForceTorqueSensors__ros_msg_type = pronto_msgs__msg__QuadrupedForceTorqueSensors;

static bool _QuadrupedForceTorqueSensors__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _QuadrupedForceTorqueSensors__ros_msg_type * ros_message = static_cast<const _QuadrupedForceTorqueSensors__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: lf
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Wrench
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->lf, cdr))
    {
      return false;
    }
  }

  // Field name: rf
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Wrench
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->rf, cdr))
    {
      return false;
    }
  }

  // Field name: lh
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Wrench
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->lh, cdr))
    {
      return false;
    }
  }

  // Field name: rh
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Wrench
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->rh, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _QuadrupedForceTorqueSensors__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _QuadrupedForceTorqueSensors__ros_msg_type * ros_message = static_cast<_QuadrupedForceTorqueSensors__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: lf
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Wrench
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->lf))
    {
      return false;
    }
  }

  // Field name: rf
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Wrench
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->rf))
    {
      return false;
    }
  }

  // Field name: lh
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Wrench
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->lh))
    {
      return false;
    }
  }

  // Field name: rh
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Wrench
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->rh))
    {
      return false;
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pronto_msgs
size_t get_serialized_size_pronto_msgs__msg__QuadrupedForceTorqueSensors(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _QuadrupedForceTorqueSensors__ros_msg_type * ros_message = static_cast<const _QuadrupedForceTorqueSensors__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name lf

  current_alignment += get_serialized_size_geometry_msgs__msg__Wrench(
    &(ros_message->lf), current_alignment);
  // field.name rf

  current_alignment += get_serialized_size_geometry_msgs__msg__Wrench(
    &(ros_message->rf), current_alignment);
  // field.name lh

  current_alignment += get_serialized_size_geometry_msgs__msg__Wrench(
    &(ros_message->lh), current_alignment);
  // field.name rh

  current_alignment += get_serialized_size_geometry_msgs__msg__Wrench(
    &(ros_message->rh), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _QuadrupedForceTorqueSensors__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_pronto_msgs__msg__QuadrupedForceTorqueSensors(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pronto_msgs
size_t max_serialized_size_pronto_msgs__msg__QuadrupedForceTorqueSensors(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: header
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_std_msgs__msg__Header(
        full_bounded, current_alignment);
    }
  }
  // member: lf
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_geometry_msgs__msg__Wrench(
        full_bounded, current_alignment);
    }
  }
  // member: rf
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_geometry_msgs__msg__Wrench(
        full_bounded, current_alignment);
    }
  }
  // member: lh
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_geometry_msgs__msg__Wrench(
        full_bounded, current_alignment);
    }
  }
  // member: rh
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_geometry_msgs__msg__Wrench(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _QuadrupedForceTorqueSensors__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_pronto_msgs__msg__QuadrupedForceTorqueSensors(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_QuadrupedForceTorqueSensors = {
  "pronto_msgs::msg",
  "QuadrupedForceTorqueSensors",
  _QuadrupedForceTorqueSensors__cdr_serialize,
  _QuadrupedForceTorqueSensors__cdr_deserialize,
  _QuadrupedForceTorqueSensors__get_serialized_size,
  _QuadrupedForceTorqueSensors__max_serialized_size
};

static rosidl_message_type_support_t _QuadrupedForceTorqueSensors__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_QuadrupedForceTorqueSensors,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pronto_msgs, msg, QuadrupedForceTorqueSensors)() {
  return &_QuadrupedForceTorqueSensors__type_support;
}

#if defined(__cplusplus)
}
#endif
