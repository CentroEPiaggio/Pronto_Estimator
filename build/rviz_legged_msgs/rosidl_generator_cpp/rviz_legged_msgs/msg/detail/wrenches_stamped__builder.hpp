// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rviz_legged_msgs:msg/WrenchesStamped.idl
// generated code does not contain a copyright notice

#ifndef RVIZ_LEGGED_MSGS__MSG__DETAIL__WRENCHES_STAMPED__BUILDER_HPP_
#define RVIZ_LEGGED_MSGS__MSG__DETAIL__WRENCHES_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rviz_legged_msgs/msg/detail/wrenches_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rviz_legged_msgs
{

namespace msg
{

namespace builder
{

class Init_WrenchesStamped_wrenches_stamped
{
public:
  explicit Init_WrenchesStamped_wrenches_stamped(::rviz_legged_msgs::msg::WrenchesStamped & msg)
  : msg_(msg)
  {}
  ::rviz_legged_msgs::msg::WrenchesStamped wrenches_stamped(::rviz_legged_msgs::msg::WrenchesStamped::_wrenches_stamped_type arg)
  {
    msg_.wrenches_stamped = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rviz_legged_msgs::msg::WrenchesStamped msg_;
};

class Init_WrenchesStamped_header
{
public:
  Init_WrenchesStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WrenchesStamped_wrenches_stamped header(::rviz_legged_msgs::msg::WrenchesStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_WrenchesStamped_wrenches_stamped(msg_);
  }

private:
  ::rviz_legged_msgs::msg::WrenchesStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rviz_legged_msgs::msg::WrenchesStamped>()
{
  return rviz_legged_msgs::msg::builder::Init_WrenchesStamped_header();
}

}  // namespace rviz_legged_msgs

#endif  // RVIZ_LEGGED_MSGS__MSG__DETAIL__WRENCHES_STAMPED__BUILDER_HPP_
