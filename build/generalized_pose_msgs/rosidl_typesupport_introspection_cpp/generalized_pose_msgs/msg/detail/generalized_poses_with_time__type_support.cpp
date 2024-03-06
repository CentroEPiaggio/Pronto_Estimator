// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from generalized_pose_msgs:msg/GeneralizedPosesWithTime.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "generalized_pose_msgs/msg/detail/generalized_poses_with_time__struct.hpp"
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

void GeneralizedPosesWithTime_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) generalized_pose_msgs::msg::GeneralizedPosesWithTime(_init);
}

void GeneralizedPosesWithTime_fini_function(void * message_memory)
{
  auto typed_message = static_cast<generalized_pose_msgs::msg::GeneralizedPosesWithTime *>(message_memory);
  typed_message->~GeneralizedPosesWithTime();
}

size_t size_function__GeneralizedPosesWithTime__generalized_poses_with_time(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<generalized_pose_msgs::msg::GeneralizedPoseWithTime> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GeneralizedPosesWithTime__generalized_poses_with_time(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<generalized_pose_msgs::msg::GeneralizedPoseWithTime> *>(untyped_member);
  return &member[index];
}

void * get_function__GeneralizedPosesWithTime__generalized_poses_with_time(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<generalized_pose_msgs::msg::GeneralizedPoseWithTime> *>(untyped_member);
  return &member[index];
}

void fetch_function__GeneralizedPosesWithTime__generalized_poses_with_time(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const generalized_pose_msgs::msg::GeneralizedPoseWithTime *>(
    get_const_function__GeneralizedPosesWithTime__generalized_poses_with_time(untyped_member, index));
  auto & value = *reinterpret_cast<generalized_pose_msgs::msg::GeneralizedPoseWithTime *>(untyped_value);
  value = item;
}

void assign_function__GeneralizedPosesWithTime__generalized_poses_with_time(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<generalized_pose_msgs::msg::GeneralizedPoseWithTime *>(
    get_function__GeneralizedPosesWithTime__generalized_poses_with_time(untyped_member, index));
  const auto & value = *reinterpret_cast<const generalized_pose_msgs::msg::GeneralizedPoseWithTime *>(untyped_value);
  item = value;
}

void resize_function__GeneralizedPosesWithTime__generalized_poses_with_time(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<generalized_pose_msgs::msg::GeneralizedPoseWithTime> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GeneralizedPosesWithTime_message_member_array[1] = {
  {
    "generalized_poses_with_time",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<generalized_pose_msgs::msg::GeneralizedPoseWithTime>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(generalized_pose_msgs::msg::GeneralizedPosesWithTime, generalized_poses_with_time),  // bytes offset in struct
    nullptr,  // default value
    size_function__GeneralizedPosesWithTime__generalized_poses_with_time,  // size() function pointer
    get_const_function__GeneralizedPosesWithTime__generalized_poses_with_time,  // get_const(index) function pointer
    get_function__GeneralizedPosesWithTime__generalized_poses_with_time,  // get(index) function pointer
    fetch_function__GeneralizedPosesWithTime__generalized_poses_with_time,  // fetch(index, &value) function pointer
    assign_function__GeneralizedPosesWithTime__generalized_poses_with_time,  // assign(index, value) function pointer
    resize_function__GeneralizedPosesWithTime__generalized_poses_with_time  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GeneralizedPosesWithTime_message_members = {
  "generalized_pose_msgs::msg",  // message namespace
  "GeneralizedPosesWithTime",  // message name
  1,  // number of fields
  sizeof(generalized_pose_msgs::msg::GeneralizedPosesWithTime),
  GeneralizedPosesWithTime_message_member_array,  // message members
  GeneralizedPosesWithTime_init_function,  // function to initialize message memory (memory has to be allocated)
  GeneralizedPosesWithTime_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GeneralizedPosesWithTime_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GeneralizedPosesWithTime_message_members,
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
get_message_type_support_handle<generalized_pose_msgs::msg::GeneralizedPosesWithTime>()
{
  return &::generalized_pose_msgs::msg::rosidl_typesupport_introspection_cpp::GeneralizedPosesWithTime_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, generalized_pose_msgs, msg, GeneralizedPosesWithTime)() {
  return &::generalized_pose_msgs::msg::rosidl_typesupport_introspection_cpp::GeneralizedPosesWithTime_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
