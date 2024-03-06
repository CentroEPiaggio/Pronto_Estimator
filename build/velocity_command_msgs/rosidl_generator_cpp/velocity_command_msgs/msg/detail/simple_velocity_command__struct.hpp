// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from velocity_command_msgs:msg/SimpleVelocityCommand.idl
// generated code does not contain a copyright notice

#ifndef VELOCITY_COMMAND_MSGS__MSG__DETAIL__SIMPLE_VELOCITY_COMMAND__STRUCT_HPP_
#define VELOCITY_COMMAND_MSGS__MSG__DETAIL__SIMPLE_VELOCITY_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__velocity_command_msgs__msg__SimpleVelocityCommand __attribute__((deprecated))
#else
# define DEPRECATED__velocity_command_msgs__msg__SimpleVelocityCommand __declspec(deprecated)
#endif

namespace velocity_command_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SimpleVelocityCommand_
{
  using Type = SimpleVelocityCommand_<ContainerAllocator>;

  explicit SimpleVelocityCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->velocity_forward = 0.0;
      this->velocity_lateral = 0.0;
      this->yaw_rate = 0.0;
    }
  }

  explicit SimpleVelocityCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->velocity_forward = 0.0;
      this->velocity_lateral = 0.0;
      this->yaw_rate = 0.0;
    }
  }

  // field types and members
  using _velocity_forward_type =
    double;
  _velocity_forward_type velocity_forward;
  using _velocity_lateral_type =
    double;
  _velocity_lateral_type velocity_lateral;
  using _yaw_rate_type =
    double;
  _yaw_rate_type yaw_rate;

  // setters for named parameter idiom
  Type & set__velocity_forward(
    const double & _arg)
  {
    this->velocity_forward = _arg;
    return *this;
  }
  Type & set__velocity_lateral(
    const double & _arg)
  {
    this->velocity_lateral = _arg;
    return *this;
  }
  Type & set__yaw_rate(
    const double & _arg)
  {
    this->yaw_rate = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    velocity_command_msgs::msg::SimpleVelocityCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const velocity_command_msgs::msg::SimpleVelocityCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<velocity_command_msgs::msg::SimpleVelocityCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<velocity_command_msgs::msg::SimpleVelocityCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      velocity_command_msgs::msg::SimpleVelocityCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<velocity_command_msgs::msg::SimpleVelocityCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      velocity_command_msgs::msg::SimpleVelocityCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<velocity_command_msgs::msg::SimpleVelocityCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<velocity_command_msgs::msg::SimpleVelocityCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<velocity_command_msgs::msg::SimpleVelocityCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__velocity_command_msgs__msg__SimpleVelocityCommand
    std::shared_ptr<velocity_command_msgs::msg::SimpleVelocityCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__velocity_command_msgs__msg__SimpleVelocityCommand
    std::shared_ptr<velocity_command_msgs::msg::SimpleVelocityCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SimpleVelocityCommand_ & other) const
  {
    if (this->velocity_forward != other.velocity_forward) {
      return false;
    }
    if (this->velocity_lateral != other.velocity_lateral) {
      return false;
    }
    if (this->yaw_rate != other.yaw_rate) {
      return false;
    }
    return true;
  }
  bool operator!=(const SimpleVelocityCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SimpleVelocityCommand_

// alias to use template instance with default allocator
using SimpleVelocityCommand =
  velocity_command_msgs::msg::SimpleVelocityCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace velocity_command_msgs

#endif  // VELOCITY_COMMAND_MSGS__MSG__DETAIL__SIMPLE_VELOCITY_COMMAND__STRUCT_HPP_
