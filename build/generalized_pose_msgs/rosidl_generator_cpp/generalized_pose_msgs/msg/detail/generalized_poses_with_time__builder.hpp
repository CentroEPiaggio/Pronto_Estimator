// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from generalized_pose_msgs:msg/GeneralizedPosesWithTime.idl
// generated code does not contain a copyright notice

#ifndef GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSES_WITH_TIME__BUILDER_HPP_
#define GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSES_WITH_TIME__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "generalized_pose_msgs/msg/detail/generalized_poses_with_time__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace generalized_pose_msgs
{

namespace msg
{

namespace builder
{

class Init_GeneralizedPosesWithTime_generalized_poses_with_time
{
public:
  Init_GeneralizedPosesWithTime_generalized_poses_with_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::generalized_pose_msgs::msg::GeneralizedPosesWithTime generalized_poses_with_time(::generalized_pose_msgs::msg::GeneralizedPosesWithTime::_generalized_poses_with_time_type arg)
  {
    msg_.generalized_poses_with_time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::generalized_pose_msgs::msg::GeneralizedPosesWithTime msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::generalized_pose_msgs::msg::GeneralizedPosesWithTime>()
{
  return generalized_pose_msgs::msg::builder::Init_GeneralizedPosesWithTime_generalized_poses_with_time();
}

}  // namespace generalized_pose_msgs

#endif  // GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSES_WITH_TIME__BUILDER_HPP_
