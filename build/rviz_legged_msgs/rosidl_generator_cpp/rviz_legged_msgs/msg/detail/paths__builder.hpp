// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rviz_legged_msgs:msg/Paths.idl
// generated code does not contain a copyright notice

#ifndef RVIZ_LEGGED_MSGS__MSG__DETAIL__PATHS__BUILDER_HPP_
#define RVIZ_LEGGED_MSGS__MSG__DETAIL__PATHS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rviz_legged_msgs/msg/detail/paths__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rviz_legged_msgs
{

namespace msg
{

namespace builder
{

class Init_Paths_paths
{
public:
  explicit Init_Paths_paths(::rviz_legged_msgs::msg::Paths & msg)
  : msg_(msg)
  {}
  ::rviz_legged_msgs::msg::Paths paths(::rviz_legged_msgs::msg::Paths::_paths_type arg)
  {
    msg_.paths = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rviz_legged_msgs::msg::Paths msg_;
};

class Init_Paths_header
{
public:
  Init_Paths_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Paths_paths header(::rviz_legged_msgs::msg::Paths::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Paths_paths(msg_);
  }

private:
  ::rviz_legged_msgs::msg::Paths msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rviz_legged_msgs::msg::Paths>()
{
  return rviz_legged_msgs::msg::builder::Init_Paths_header();
}

}  // namespace rviz_legged_msgs

#endif  // RVIZ_LEGGED_MSGS__MSG__DETAIL__PATHS__BUILDER_HPP_
