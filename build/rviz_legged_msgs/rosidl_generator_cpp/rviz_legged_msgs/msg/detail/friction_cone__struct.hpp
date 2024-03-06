// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rviz_legged_msgs:msg/FrictionCone.idl
// generated code does not contain a copyright notice

#ifndef RVIZ_LEGGED_MSGS__MSG__DETAIL__FRICTION_CONE__STRUCT_HPP_
#define RVIZ_LEGGED_MSGS__MSG__DETAIL__FRICTION_CONE__STRUCT_HPP_

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
// Member 'normal_direction'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rviz_legged_msgs__msg__FrictionCone __attribute__((deprecated))
#else
# define DEPRECATED__rviz_legged_msgs__msg__FrictionCone __declspec(deprecated)
#endif

namespace rviz_legged_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FrictionCone_
{
  using Type = FrictionCone_<ContainerAllocator>;

  explicit FrictionCone_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    normal_direction(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->friction_coefficient = 0.0;
    }
  }

  explicit FrictionCone_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    normal_direction(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->friction_coefficient = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _normal_direction_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _normal_direction_type normal_direction;
  using _friction_coefficient_type =
    double;
  _friction_coefficient_type friction_coefficient;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__normal_direction(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->normal_direction = _arg;
    return *this;
  }
  Type & set__friction_coefficient(
    const double & _arg)
  {
    this->friction_coefficient = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rviz_legged_msgs::msg::FrictionCone_<ContainerAllocator> *;
  using ConstRawPtr =
    const rviz_legged_msgs::msg::FrictionCone_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rviz_legged_msgs::msg::FrictionCone_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rviz_legged_msgs::msg::FrictionCone_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rviz_legged_msgs::msg::FrictionCone_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rviz_legged_msgs::msg::FrictionCone_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rviz_legged_msgs::msg::FrictionCone_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rviz_legged_msgs::msg::FrictionCone_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rviz_legged_msgs::msg::FrictionCone_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rviz_legged_msgs::msg::FrictionCone_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rviz_legged_msgs__msg__FrictionCone
    std::shared_ptr<rviz_legged_msgs::msg::FrictionCone_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rviz_legged_msgs__msg__FrictionCone
    std::shared_ptr<rviz_legged_msgs::msg::FrictionCone_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FrictionCone_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->normal_direction != other.normal_direction) {
      return false;
    }
    if (this->friction_coefficient != other.friction_coefficient) {
      return false;
    }
    return true;
  }
  bool operator!=(const FrictionCone_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FrictionCone_

// alias to use template instance with default allocator
using FrictionCone =
  rviz_legged_msgs::msg::FrictionCone_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rviz_legged_msgs

#endif  // RVIZ_LEGGED_MSGS__MSG__DETAIL__FRICTION_CONE__STRUCT_HPP_
