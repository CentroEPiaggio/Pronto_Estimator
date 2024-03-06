// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from pi3hat_moteus_int_msgs:msg/JointsStates.idl
// generated code does not contain a copyright notice

#ifndef PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__JOINTS_STATES__STRUCT_HPP_
#define PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__JOINTS_STATES__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__pi3hat_moteus_int_msgs__msg__JointsStates __attribute__((deprecated))
#else
# define DEPRECATED__pi3hat_moteus_int_msgs__msg__JointsStates __declspec(deprecated)
#endif

namespace pi3hat_moteus_int_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct JointsStates_
{
  using Type = JointsStates_<ContainerAllocator>;

  explicit JointsStates_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit JointsStates_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _name_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _name_type name;
  using _position_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _position_type position;
  using _velocity_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _velocity_type velocity;
  using _effort_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _effort_type effort;
  using _current_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _current_type current;
  using _temperature_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _temperature_type temperature;
  using _sec_enc_pos_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _sec_enc_pos_type sec_enc_pos;
  using _sec_enc_vel_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _sec_enc_vel_type sec_enc_vel;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__name(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__position(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__velocity(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__effort(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->effort = _arg;
    return *this;
  }
  Type & set__current(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->current = _arg;
    return *this;
  }
  Type & set__temperature(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__sec_enc_pos(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->sec_enc_pos = _arg;
    return *this;
  }
  Type & set__sec_enc_vel(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->sec_enc_vel = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pi3hat_moteus_int_msgs::msg::JointsStates_<ContainerAllocator> *;
  using ConstRawPtr =
    const pi3hat_moteus_int_msgs::msg::JointsStates_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pi3hat_moteus_int_msgs::msg::JointsStates_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pi3hat_moteus_int_msgs::msg::JointsStates_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pi3hat_moteus_int_msgs::msg::JointsStates_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pi3hat_moteus_int_msgs::msg::JointsStates_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pi3hat_moteus_int_msgs::msg::JointsStates_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pi3hat_moteus_int_msgs::msg::JointsStates_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pi3hat_moteus_int_msgs::msg::JointsStates_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pi3hat_moteus_int_msgs::msg::JointsStates_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pi3hat_moteus_int_msgs__msg__JointsStates
    std::shared_ptr<pi3hat_moteus_int_msgs::msg::JointsStates_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pi3hat_moteus_int_msgs__msg__JointsStates
    std::shared_ptr<pi3hat_moteus_int_msgs::msg::JointsStates_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JointsStates_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->name != other.name) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->effort != other.effort) {
      return false;
    }
    if (this->current != other.current) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->sec_enc_pos != other.sec_enc_pos) {
      return false;
    }
    if (this->sec_enc_vel != other.sec_enc_vel) {
      return false;
    }
    return true;
  }
  bool operator!=(const JointsStates_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JointsStates_

// alias to use template instance with default allocator
using JointsStates =
  pi3hat_moteus_int_msgs::msg::JointsStates_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace pi3hat_moteus_int_msgs

#endif  // PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__JOINTS_STATES__STRUCT_HPP_
