// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from generalized_pose_msgs:msg/GeneralizedPose.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "generalized_pose_msgs/msg/detail/generalized_pose__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace generalized_pose_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void GeneralizedPose_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) generalized_pose_msgs::msg::GeneralizedPose(_init);
}

void GeneralizedPose_fini_function(void * message_memory)
{
  auto typed_message = static_cast<generalized_pose_msgs::msg::GeneralizedPose *>(message_memory);
  typed_message->~GeneralizedPose();
}

size_t size_function__GeneralizedPose__feet_acc(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GeneralizedPose__feet_acc(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__GeneralizedPose__feet_acc(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__GeneralizedPose__feet_acc(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__GeneralizedPose__feet_acc(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__GeneralizedPose__feet_acc(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__GeneralizedPose__feet_acc(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__GeneralizedPose__feet_acc(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GeneralizedPose__feet_vel(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GeneralizedPose__feet_vel(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__GeneralizedPose__feet_vel(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__GeneralizedPose__feet_vel(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__GeneralizedPose__feet_vel(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__GeneralizedPose__feet_vel(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__GeneralizedPose__feet_vel(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__GeneralizedPose__feet_vel(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GeneralizedPose__feet_pos(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GeneralizedPose__feet_pos(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__GeneralizedPose__feet_pos(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__GeneralizedPose__feet_pos(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__GeneralizedPose__feet_pos(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__GeneralizedPose__feet_pos(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__GeneralizedPose__feet_pos(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__GeneralizedPose__feet_pos(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GeneralizedPose__contact_feet(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GeneralizedPose__contact_feet(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__GeneralizedPose__contact_feet(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__GeneralizedPose__contact_feet(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__GeneralizedPose__contact_feet(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__GeneralizedPose__contact_feet(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__GeneralizedPose__contact_feet(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__GeneralizedPose__contact_feet(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GeneralizedPose_message_member_array[9] = {
  {
    "base_acc",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Vector3>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(generalized_pose_msgs::msg::GeneralizedPose, base_acc),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "base_vel",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Vector3>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(generalized_pose_msgs::msg::GeneralizedPose, base_vel),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "base_pos",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Vector3>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(generalized_pose_msgs::msg::GeneralizedPose, base_pos),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "base_angvel",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Vector3>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(generalized_pose_msgs::msg::GeneralizedPose, base_angvel),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "base_quat",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Quaternion>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(generalized_pose_msgs::msg::GeneralizedPose, base_quat),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "feet_acc",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(generalized_pose_msgs::msg::GeneralizedPose, feet_acc),  // bytes offset in struct
    nullptr,  // default value
    size_function__GeneralizedPose__feet_acc,  // size() function pointer
    get_const_function__GeneralizedPose__feet_acc,  // get_const(index) function pointer
    get_function__GeneralizedPose__feet_acc,  // get(index) function pointer
    fetch_function__GeneralizedPose__feet_acc,  // fetch(index, &value) function pointer
    assign_function__GeneralizedPose__feet_acc,  // assign(index, value) function pointer
    resize_function__GeneralizedPose__feet_acc  // resize(index) function pointer
  },
  {
    "feet_vel",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(generalized_pose_msgs::msg::GeneralizedPose, feet_vel),  // bytes offset in struct
    nullptr,  // default value
    size_function__GeneralizedPose__feet_vel,  // size() function pointer
    get_const_function__GeneralizedPose__feet_vel,  // get_const(index) function pointer
    get_function__GeneralizedPose__feet_vel,  // get(index) function pointer
    fetch_function__GeneralizedPose__feet_vel,  // fetch(index, &value) function pointer
    assign_function__GeneralizedPose__feet_vel,  // assign(index, value) function pointer
    resize_function__GeneralizedPose__feet_vel  // resize(index) function pointer
  },
  {
    "feet_pos",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(generalized_pose_msgs::msg::GeneralizedPose, feet_pos),  // bytes offset in struct
    nullptr,  // default value
    size_function__GeneralizedPose__feet_pos,  // size() function pointer
    get_const_function__GeneralizedPose__feet_pos,  // get_const(index) function pointer
    get_function__GeneralizedPose__feet_pos,  // get(index) function pointer
    fetch_function__GeneralizedPose__feet_pos,  // fetch(index, &value) function pointer
    assign_function__GeneralizedPose__feet_pos,  // assign(index, value) function pointer
    resize_function__GeneralizedPose__feet_pos  // resize(index) function pointer
  },
  {
    "contact_feet",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(generalized_pose_msgs::msg::GeneralizedPose, contact_feet),  // bytes offset in struct
    nullptr,  // default value
    size_function__GeneralizedPose__contact_feet,  // size() function pointer
    get_const_function__GeneralizedPose__contact_feet,  // get_const(index) function pointer
    get_function__GeneralizedPose__contact_feet,  // get(index) function pointer
    fetch_function__GeneralizedPose__contact_feet,  // fetch(index, &value) function pointer
    assign_function__GeneralizedPose__contact_feet,  // assign(index, value) function pointer
    resize_function__GeneralizedPose__contact_feet  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GeneralizedPose_message_members = {
  "generalized_pose_msgs::msg",  // message namespace
  "GeneralizedPose",  // message name
  9,  // number of fields
  sizeof(generalized_pose_msgs::msg::GeneralizedPose),
  GeneralizedPose_message_member_array,  // message members
  GeneralizedPose_init_function,  // function to initialize message memory (memory has to be allocated)
  GeneralizedPose_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GeneralizedPose_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GeneralizedPose_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace generalized_pose_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<generalized_pose_msgs::msg::GeneralizedPose>()
{
  return &::generalized_pose_msgs::msg::rosidl_typesupport_introspection_cpp::GeneralizedPose_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, generalized_pose_msgs, msg, GeneralizedPose)() {
  return &::generalized_pose_msgs::msg::rosidl_typesupport_introspection_cpp::GeneralizedPose_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
