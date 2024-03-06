// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from pronto_msgs:msg/QuadrupedStance.idl
// generated code does not contain a copyright notice
#include "pronto_msgs/msg/detail/quadruped_stance__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "pronto_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "pronto_msgs/msg/detail/quadruped_stance__struct.h"
#include "pronto_msgs/msg/detail/quadruped_stance__functions.h"
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

#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions
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


using _QuadrupedStance__ros_msg_type = pronto_msgs__msg__QuadrupedStance;

static bool _QuadrupedStance__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _QuadrupedStance__ros_msg_type * ros_message = static_cast<const _QuadrupedStance__ros_msg_type *>(untyped_ros_message);
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
    cdr << ros_message->lf;
  }

  // Field name: rf
  {
    cdr << ros_message->rf;
  }

  // Field name: lh
  {
    cdr << ros_message->lh;
  }

  // Field name: rh
  {
    cdr << ros_message->rh;
  }

  return true;
}

static bool _QuadrupedStance__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _QuadrupedStance__ros_msg_type * ros_message = static_cast<_QuadrupedStance__ros_msg_type *>(untyped_ros_message);
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
    cdr >> ros_message->lf;
  }

  // Field name: rf
  {
    cdr >> ros_message->rf;
  }

  // Field name: lh
  {
    cdr >> ros_message->lh;
  }

  // Field name: rh
  {
    cdr >> ros_message->rh;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pronto_msgs
size_t get_serialized_size_pronto_msgs__msg__QuadrupedStance(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _QuadrupedStance__ros_msg_type * ros_message = static_cast<const _QuadrupedStance__ros_msg_type *>(untyped_ros_message);
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
  {
    size_t item_size = sizeof(ros_message->lf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rf
  {
    size_t item_size = sizeof(ros_message->rf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name lh
  {
    size_t item_size = sizeof(ros_message->lh);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rh
  {
    size_t item_size = sizeof(ros_message->rh);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _QuadrupedStance__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_pronto_msgs__msg__QuadrupedStance(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pronto_msgs
size_t max_serialized_size_pronto_msgs__msg__QuadrupedStance(
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

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: rf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: lh
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: rh
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _QuadrupedStance__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_pronto_msgs__msg__QuadrupedStance(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_QuadrupedStance = {
  "pronto_msgs::msg",
  "QuadrupedStance",
  _QuadrupedStance__cdr_serialize,
  _QuadrupedStance__cdr_deserialize,
  _QuadrupedStance__get_serialized_size,
  _QuadrupedStance__max_serialized_size
};

static rosidl_message_type_support_t _QuadrupedStance__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_QuadrupedStance,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pronto_msgs, msg, QuadrupedStance)() {
  return &_QuadrupedStance__type_support;
}

#if defined(__cplusplus)
}
#endif
