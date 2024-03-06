// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from generalized_pose_msgs:msg/GeneralizedPose.idl
// generated code does not contain a copyright notice
#include "generalized_pose_msgs/msg/detail/generalized_pose__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "generalized_pose_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "generalized_pose_msgs/msg/detail/generalized_pose__struct.h"
#include "generalized_pose_msgs/msg/detail/generalized_pose__functions.h"
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

#include "geometry_msgs/msg/detail/quaternion__functions.h"  // base_quat
#include "geometry_msgs/msg/detail/vector3__functions.h"  // base_acc, base_angvel, base_pos, base_vel
#include "rosidl_runtime_c/primitives_sequence.h"  // feet_acc, feet_pos, feet_vel
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // feet_acc, feet_pos, feet_vel
#include "rosidl_runtime_c/string.h"  // contact_feet
#include "rosidl_runtime_c/string_functions.h"  // contact_feet

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_generalized_pose_msgs
size_t get_serialized_size_geometry_msgs__msg__Quaternion(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_generalized_pose_msgs
size_t max_serialized_size_geometry_msgs__msg__Quaternion(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_generalized_pose_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Quaternion)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_generalized_pose_msgs
size_t get_serialized_size_geometry_msgs__msg__Vector3(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_generalized_pose_msgs
size_t max_serialized_size_geometry_msgs__msg__Vector3(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_generalized_pose_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Vector3)();


using _GeneralizedPose__ros_msg_type = generalized_pose_msgs__msg__GeneralizedPose;

static bool _GeneralizedPose__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GeneralizedPose__ros_msg_type * ros_message = static_cast<const _GeneralizedPose__ros_msg_type *>(untyped_ros_message);
  // Field name: base_acc
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Vector3
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->base_acc, cdr))
    {
      return false;
    }
  }

  // Field name: base_vel
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Vector3
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->base_vel, cdr))
    {
      return false;
    }
  }

  // Field name: base_pos
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Vector3
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->base_pos, cdr))
    {
      return false;
    }
  }

  // Field name: base_angvel
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Vector3
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->base_angvel, cdr))
    {
      return false;
    }
  }

  // Field name: base_quat
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Quaternion
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->base_quat, cdr))
    {
      return false;
    }
  }

  // Field name: feet_acc
  {
    size_t size = ros_message->feet_acc.size;
    auto array_ptr = ros_message->feet_acc.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: feet_vel
  {
    size_t size = ros_message->feet_vel.size;
    auto array_ptr = ros_message->feet_vel.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: feet_pos
  {
    size_t size = ros_message->feet_pos.size;
    auto array_ptr = ros_message->feet_pos.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: contact_feet
  {
    size_t size = ros_message->contact_feet.size;
    auto array_ptr = ros_message->contact_feet.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      const rosidl_runtime_c__String * str = &array_ptr[i];
      if (str->capacity == 0 || str->capacity <= str->size) {
        fprintf(stderr, "string capacity not greater than size\n");
        return false;
      }
      if (str->data[str->size] != '\0') {
        fprintf(stderr, "string not null-terminated\n");
        return false;
      }
      cdr << str->data;
    }
  }

  return true;
}

static bool _GeneralizedPose__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GeneralizedPose__ros_msg_type * ros_message = static_cast<_GeneralizedPose__ros_msg_type *>(untyped_ros_message);
  // Field name: base_acc
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Vector3
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->base_acc))
    {
      return false;
    }
  }

  // Field name: base_vel
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Vector3
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->base_vel))
    {
      return false;
    }
  }

  // Field name: base_pos
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Vector3
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->base_pos))
    {
      return false;
    }
  }

  // Field name: base_angvel
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Vector3
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->base_angvel))
    {
      return false;
    }
  }

  // Field name: base_quat
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Quaternion
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->base_quat))
    {
      return false;
    }
  }

  // Field name: feet_acc
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->feet_acc.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->feet_acc);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->feet_acc, size)) {
      fprintf(stderr, "failed to create array for field 'feet_acc'");
      return false;
    }
    auto array_ptr = ros_message->feet_acc.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: feet_vel
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->feet_vel.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->feet_vel);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->feet_vel, size)) {
      fprintf(stderr, "failed to create array for field 'feet_vel'");
      return false;
    }
    auto array_ptr = ros_message->feet_vel.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: feet_pos
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->feet_pos.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->feet_pos);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->feet_pos, size)) {
      fprintf(stderr, "failed to create array for field 'feet_pos'");
      return false;
    }
    auto array_ptr = ros_message->feet_pos.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: contact_feet
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->contact_feet.data) {
      rosidl_runtime_c__String__Sequence__fini(&ros_message->contact_feet);
    }
    if (!rosidl_runtime_c__String__Sequence__init(&ros_message->contact_feet, size)) {
      fprintf(stderr, "failed to create array for field 'contact_feet'");
      return false;
    }
    auto array_ptr = ros_message->contact_feet.data;
    for (size_t i = 0; i < size; ++i) {
      std::string tmp;
      cdr >> tmp;
      auto & ros_i = array_ptr[i];
      if (!ros_i.data) {
        rosidl_runtime_c__String__init(&ros_i);
      }
      bool succeeded = rosidl_runtime_c__String__assign(
        &ros_i,
        tmp.c_str());
      if (!succeeded) {
        fprintf(stderr, "failed to assign string into field 'contact_feet'\n");
        return false;
      }
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_generalized_pose_msgs
size_t get_serialized_size_generalized_pose_msgs__msg__GeneralizedPose(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GeneralizedPose__ros_msg_type * ros_message = static_cast<const _GeneralizedPose__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name base_acc

  current_alignment += get_serialized_size_geometry_msgs__msg__Vector3(
    &(ros_message->base_acc), current_alignment);
  // field.name base_vel

  current_alignment += get_serialized_size_geometry_msgs__msg__Vector3(
    &(ros_message->base_vel), current_alignment);
  // field.name base_pos

  current_alignment += get_serialized_size_geometry_msgs__msg__Vector3(
    &(ros_message->base_pos), current_alignment);
  // field.name base_angvel

  current_alignment += get_serialized_size_geometry_msgs__msg__Vector3(
    &(ros_message->base_angvel), current_alignment);
  // field.name base_quat

  current_alignment += get_serialized_size_geometry_msgs__msg__Quaternion(
    &(ros_message->base_quat), current_alignment);
  // field.name feet_acc
  {
    size_t array_size = ros_message->feet_acc.size;
    auto array_ptr = ros_message->feet_acc.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name feet_vel
  {
    size_t array_size = ros_message->feet_vel.size;
    auto array_ptr = ros_message->feet_vel.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name feet_pos
  {
    size_t array_size = ros_message->feet_pos.size;
    auto array_ptr = ros_message->feet_pos.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name contact_feet
  {
    size_t array_size = ros_message->contact_feet.size;
    auto array_ptr = ros_message->contact_feet.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (array_ptr[index].size + 1);
    }
  }

  return current_alignment - initial_alignment;
}

static uint32_t _GeneralizedPose__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_generalized_pose_msgs__msg__GeneralizedPose(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_generalized_pose_msgs
size_t max_serialized_size_generalized_pose_msgs__msg__GeneralizedPose(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: base_acc
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: base_vel
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: base_pos
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: base_angvel
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: base_quat
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__Quaternion(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: feet_acc
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: feet_vel
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: feet_pos
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: contact_feet
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = generalized_pose_msgs__msg__GeneralizedPose;
    is_plain =
      (
      offsetof(DataType, contact_feet) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GeneralizedPose__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_generalized_pose_msgs__msg__GeneralizedPose(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GeneralizedPose = {
  "generalized_pose_msgs::msg",
  "GeneralizedPose",
  _GeneralizedPose__cdr_serialize,
  _GeneralizedPose__cdr_deserialize,
  _GeneralizedPose__get_serialized_size,
  _GeneralizedPose__max_serialized_size
};

static rosidl_message_type_support_t _GeneralizedPose__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GeneralizedPose,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, generalized_pose_msgs, msg, GeneralizedPose)() {
  return &_GeneralizedPose__type_support;
}

#if defined(__cplusplus)
}
#endif
