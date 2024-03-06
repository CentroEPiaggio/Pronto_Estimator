// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from pronto_msgs:msg/BipedForceTorqueSensors.idl
// generated code does not contain a copyright notice
#include "pronto_msgs/msg/detail/biped_force_torque_sensors__rosidl_typesupport_fastrtps_cpp.hpp"
#include "pronto_msgs/msg/detail/biped_force_torque_sensors__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs

namespace geometry_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const geometry_msgs::msg::Wrench &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  geometry_msgs::msg::Wrench &);
size_t get_serialized_size(
  const geometry_msgs::msg::Wrench &,
  size_t current_alignment);
size_t
max_serialized_size_Wrench(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace geometry_msgs

namespace geometry_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const geometry_msgs::msg::Wrench &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  geometry_msgs::msg::Wrench &);
size_t get_serialized_size(
  const geometry_msgs::msg::Wrench &,
  size_t current_alignment);
size_t
max_serialized_size_Wrench(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace geometry_msgs

namespace geometry_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const geometry_msgs::msg::Wrench &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  geometry_msgs::msg::Wrench &);
size_t get_serialized_size(
  const geometry_msgs::msg::Wrench &,
  size_t current_alignment);
size_t
max_serialized_size_Wrench(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace geometry_msgs

namespace geometry_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const geometry_msgs::msg::Wrench &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  geometry_msgs::msg::Wrench &);
size_t get_serialized_size(
  const geometry_msgs::msg::Wrench &,
  size_t current_alignment);
size_t
max_serialized_size_Wrench(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace geometry_msgs


namespace pronto_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_pronto_msgs
cdr_serialize(
  const pronto_msgs::msg::BipedForceTorqueSensors & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: l_foot
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.l_foot,
    cdr);
  // Member: r_foot
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.r_foot,
    cdr);
  // Member: l_hand
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.l_hand,
    cdr);
  // Member: r_hand
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.r_hand,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_pronto_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  pronto_msgs::msg::BipedForceTorqueSensors & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: l_foot
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.l_foot);

  // Member: r_foot
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.r_foot);

  // Member: l_hand
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.l_hand);

  // Member: r_hand
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.r_hand);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_pronto_msgs
get_serialized_size(
  const pronto_msgs::msg::BipedForceTorqueSensors & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: l_foot

  current_alignment +=
    geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.l_foot, current_alignment);
  // Member: r_foot

  current_alignment +=
    geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.r_foot, current_alignment);
  // Member: l_hand

  current_alignment +=
    geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.l_hand, current_alignment);
  // Member: r_hand

  current_alignment +=
    geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.r_hand, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_pronto_msgs
max_serialized_size_BipedForceTorqueSensors(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: header
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        full_bounded, current_alignment);
    }
  }

  // Member: l_foot
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Wrench(
        full_bounded, current_alignment);
    }
  }

  // Member: r_foot
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Wrench(
        full_bounded, current_alignment);
    }
  }

  // Member: l_hand
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Wrench(
        full_bounded, current_alignment);
    }
  }

  // Member: r_hand
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Wrench(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static bool _BipedForceTorqueSensors__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const pronto_msgs::msg::BipedForceTorqueSensors *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _BipedForceTorqueSensors__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<pronto_msgs::msg::BipedForceTorqueSensors *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _BipedForceTorqueSensors__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const pronto_msgs::msg::BipedForceTorqueSensors *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _BipedForceTorqueSensors__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_BipedForceTorqueSensors(full_bounded, 0);
}

static message_type_support_callbacks_t _BipedForceTorqueSensors__callbacks = {
  "pronto_msgs::msg",
  "BipedForceTorqueSensors",
  _BipedForceTorqueSensors__cdr_serialize,
  _BipedForceTorqueSensors__cdr_deserialize,
  _BipedForceTorqueSensors__get_serialized_size,
  _BipedForceTorqueSensors__max_serialized_size
};

static rosidl_message_type_support_t _BipedForceTorqueSensors__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_BipedForceTorqueSensors__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace pronto_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_pronto_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<pronto_msgs::msg::BipedForceTorqueSensors>()
{
  return &pronto_msgs::msg::typesupport_fastrtps_cpp::_BipedForceTorqueSensors__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, pronto_msgs, msg, BipedForceTorqueSensors)() {
  return &pronto_msgs::msg::typesupport_fastrtps_cpp::_BipedForceTorqueSensors__handle;
}

#ifdef __cplusplus
}
#endif
