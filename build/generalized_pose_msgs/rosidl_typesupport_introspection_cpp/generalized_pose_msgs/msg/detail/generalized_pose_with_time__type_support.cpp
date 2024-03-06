// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from generalized_pose_msgs:msg/GeneralizedPoseWithTime.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "generalized_pose_msgs/msg/detail/generalized_pose_with_time__struct.hpp"
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

void GeneralizedPoseWithTime_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) generalized_pose_msgs::msg::GeneralizedPoseWithTime(_init);
}

void GeneralizedPoseWithTime_fini_function(void * message_memory)
{
  auto typed_message = static_cast<generalized_pose_msgs::msg::GeneralizedPoseWithTime *>(message_memory);
  typed_message->~GeneralizedPoseWithTime();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GeneralizedPoseWithTime_message_member_array[2] = {
  {
    "generalized_pose",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<generalized_pose_msgs::msg::GeneralizedPose>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(generalized_pose_msgs::msg::GeneralizedPoseWithTime, generalized_pose),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "time",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(generalized_pose_msgs::msg::GeneralizedPoseWithTime, time),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GeneralizedPoseWithTime_message_members = {
  "generalized_pose_msgs::msg",  // message namespace
  "GeneralizedPoseWithTime",  // message name
  2,  // number of fields
  sizeof(generalized_pose_msgs::msg::GeneralizedPoseWithTime),
  GeneralizedPoseWithTime_message_member_array,  // message members
  GeneralizedPoseWithTime_init_function,  // function to initialize message memory (memory has to be allocated)
  GeneralizedPoseWithTime_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GeneralizedPoseWithTime_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GeneralizedPoseWithTime_message_members,
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
get_message_type_support_handle<generalized_pose_msgs::msg::GeneralizedPoseWithTime>()
{
  return &::generalized_pose_msgs::msg::rosidl_typesupport_introspection_cpp::GeneralizedPoseWithTime_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, generalized_pose_msgs, msg, GeneralizedPoseWithTime)() {
  return &::generalized_pose_msgs::msg::rosidl_typesupport_introspection_cpp::GeneralizedPoseWithTime_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
