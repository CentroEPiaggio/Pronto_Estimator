// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rviz_legged_msgs:msg/FrictionCones.idl
// generated code does not contain a copyright notice

#ifndef RVIZ_LEGGED_MSGS__MSG__DETAIL__FRICTION_CONES__STRUCT_HPP_
#define RVIZ_LEGGED_MSGS__MSG__DETAIL__FRICTION_CONES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'friction_cones'
#include "rviz_legged_msgs/msg/detail/friction_cone__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rviz_legged_msgs__msg__FrictionCones __attribute__((deprecated))
#else
# define DEPRECATED__rviz_legged_msgs__msg__FrictionCones __declspec(deprecated)
#endif

namespace rviz_legged_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FrictionCones_
{
  using Type = FrictionCones_<ContainerAllocator>;

  explicit FrictionCones_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit FrictionCones_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _friction_cones_type =
    std::vector<rviz_legged_msgs::msg::FrictionCone_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<rviz_legged_msgs::msg::FrictionCone_<ContainerAllocator>>>;
  _friction_cones_type friction_cones;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__friction_cones(
    const std::vector<rviz_legged_msgs::msg::FrictionCone_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<rviz_legged_msgs::msg::FrictionCone_<ContainerAllocator>>> & _arg)
  {
    this->friction_cones = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rviz_legged_msgs::msg::FrictionCones_<ContainerAllocator> *;
  using ConstRawPtr =
    const rviz_legged_msgs::msg::FrictionCones_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rviz_legged_msgs::msg::FrictionCones_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rviz_legged_msgs::msg::FrictionCones_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rviz_legged_msgs::msg::FrictionCones_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rviz_legged_msgs::msg::FrictionCones_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rviz_legged_msgs::msg::FrictionCones_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rviz_legged_msgs::msg::FrictionCones_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rviz_legged_msgs::msg::FrictionCones_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rviz_legged_msgs::msg::FrictionCones_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rviz_legged_msgs__msg__FrictionCones
    std::shared_ptr<rviz_legged_msgs::msg::FrictionCones_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rviz_legged_msgs__msg__FrictionCones
    std::shared_ptr<rviz_legged_msgs::msg::FrictionCones_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FrictionCones_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->friction_cones != other.friction_cones) {
      return false;
    }
    return true;
  }
  bool operator!=(const FrictionCones_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FrictionCones_

// alias to use template instance with default allocator
using FrictionCones =
  rviz_legged_msgs::msg::FrictionCones_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rviz_legged_msgs

#endif  // RVIZ_LEGGED_MSGS__MSG__DETAIL__FRICTION_CONES__STRUCT_HPP_
