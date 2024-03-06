// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from generalized_pose_msgs:msg/GeneralizedPoseWithTime.idl
// generated code does not contain a copyright notice

#ifndef GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSE_WITH_TIME__BUILDER_HPP_
#define GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSE_WITH_TIME__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "generalized_pose_msgs/msg/detail/generalized_pose_with_time__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace generalized_pose_msgs
{

namespace msg
{

namespace builder
{

class Init_GeneralizedPoseWithTime_time
{
public:
  explicit Init_GeneralizedPoseWithTime_time(::generalized_pose_msgs::msg::GeneralizedPoseWithTime & msg)
  : msg_(msg)
  {}
  ::generalized_pose_msgs::msg::GeneralizedPoseWithTime time(::generalized_pose_msgs::msg::GeneralizedPoseWithTime::_time_type arg)
  {
    msg_.time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::generalized_pose_msgs::msg::GeneralizedPoseWithTime msg_;
};

class Init_GeneralizedPoseWithTime_generalized_pose
{
public:
  Init_GeneralizedPoseWithTime_generalized_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GeneralizedPoseWithTime_time generalized_pose(::generalized_pose_msgs::msg::GeneralizedPoseWithTime::_generalized_pose_type arg)
  {
    msg_.generalized_pose = std::move(arg);
    return Init_GeneralizedPoseWithTime_time(msg_);
  }

private:
  ::generalized_pose_msgs::msg::GeneralizedPoseWithTime msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::generalized_pose_msgs::msg::GeneralizedPoseWithTime>()
{
  return generalized_pose_msgs::msg::builder::Init_GeneralizedPoseWithTime_generalized_pose();
}

}  // namespace generalized_pose_msgs

#endif  // GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSE_WITH_TIME__BUILDER_HPP_
