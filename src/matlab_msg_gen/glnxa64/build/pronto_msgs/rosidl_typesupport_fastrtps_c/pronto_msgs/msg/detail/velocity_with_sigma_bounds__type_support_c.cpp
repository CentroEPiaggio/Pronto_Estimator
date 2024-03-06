// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from pronto_msgs:msg/VelocityWithSigmaBounds.idl
// generated code does not contain a copyright notice
#include "pronto_msgs/msg/detail/velocity_with_sigma_bounds__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "pronto_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "pronto_msgs/msg/detail/velocity_with_sigma_bounds__struct.h"
#include "pronto_msgs/msg/detail/velocity_with_sigma_bounds__functions.h"
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

#include "geometry_msgs/msg/detail/vector3__functions.h"  // plus_one_sigma, velocity_minus_one_sigma, velocity_plus_one_sigma
#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pronto_msgs
size_t get_serialized_size_geometry_msgs__msg__Vector3(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pronto_msgs
size_t max_serialized_size_geometry_msgs__msg__Vector3(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pronto_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Vector3)();
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


using _VelocityWithSigmaBounds__ros_msg_type = pronto_msgs__msg__VelocityWithSigmaBounds;

static bool _VelocityWithSigmaBounds__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _VelocityWithSigmaBounds__ros_msg_type * ros_message = static_cast<const _VelocityWithSigmaBounds__ros_msg_type *>(untyped_ros_message);
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

  // Field name: velocity_plus_one_sigma
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Vector3
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->velocity_plus_one_sigma, cdr))
    {
      return false;
    }
  }

  // Field name: velocity_minus_one_sigma
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Vector3
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->velocity_minus_one_sigma, cdr))
    {
      return false;
    }
  }

  // Field name: plus_one_sigma
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Vector3
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->plus_one_sigma, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _VelocityWithSigmaBounds__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _VelocityWithSigmaBounds__ros_msg_type * ros_message = static_cast<_VelocityWithSigmaBounds__ros_msg_type *>(untyped_ros_message);
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

  // Field name: velocity_plus_one_sigma
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Vector3
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->velocity_plus_one_sigma))
    {
      return false;
    }
  }

  // Field name: velocity_minus_one_sigma
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Vector3
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->velocity_minus_one_sigma))
    {
      return false;
    }
  }

  // Field name: plus_one_sigma
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Vector3
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->plus_one_sigma))
    {
      return false;
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pronto_msgs
size_t get_serialized_size_pronto_msgs__msg__VelocityWithSigmaBounds(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _VelocityWithSigmaBounds__ros_msg_type * ros_message = static_cast<const _VelocityWithSigmaBounds__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name velocity_plus_one_sigma

  current_alignment += get_serialized_size_geometry_msgs__msg__Vector3(
    &(ros_message->velocity_plus_one_sigma), current_alignment);
  // field.name velocity_minus_one_sigma

  current_alignment += get_serialized_size_geometry_msgs__msg__Vector3(
    &(ros_message->velocity_minus_one_sigma), current_alignment);
  // field.name plus_one_sigma

  current_alignment += get_serialized_size_geometry_msgs__msg__Vector3(
    &(ros_message->plus_one_sigma), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _VelocityWithSigmaBounds__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_pronto_msgs__msg__VelocityWithSigmaBounds(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pronto_msgs
size_t max_serialized_size_pronto_msgs__msg__VelocityWithSigmaBounds(
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
  // member: velocity_plus_one_sigma
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_geometry_msgs__msg__Vector3(
        full_bounded, current_alignment);
    }
  }
  // member: velocity_minus_one_sigma
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_geometry_msgs__msg__Vector3(
        full_bounded, current_alignment);
    }
  }
  // member: plus_one_sigma
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_geometry_msgs__msg__Vector3(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _VelocityWithSigmaBounds__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_pronto_msgs__msg__VelocityWithSigmaBounds(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_VelocityWithSigmaBounds = {
  "pronto_msgs::msg",
  "VelocityWithSigmaBounds",
  _VelocityWithSigmaBounds__cdr_serialize,
  _VelocityWithSigmaBounds__cdr_deserialize,
  _VelocityWithSigmaBounds__get_serialized_size,
  _VelocityWithSigmaBounds__max_serialized_size
};

static rosidl_message_type_support_t _VelocityWithSigmaBounds__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_VelocityWithSigmaBounds,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pronto_msgs, msg, VelocityWithSigmaBounds)() {
  return &_VelocityWithSigmaBounds__type_support;
}

#if defined(__cplusplus)
}
#endif
