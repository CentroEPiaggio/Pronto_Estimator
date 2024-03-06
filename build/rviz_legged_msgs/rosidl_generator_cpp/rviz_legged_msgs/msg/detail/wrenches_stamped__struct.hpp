// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rviz_legged_msgs:msg/WrenchesStamped.idl
// generated code does not contain a copyright notice

#ifndef RVIZ_LEGGED_MSGS__MSG__DETAIL__WRENCHES_STAMPED__STRUCT_HPP_
#define RVIZ_LEGGED_MSGS__MSG__DETAIL__WRENCHES_STAMPED__STRUCT_HPP_

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
// Member 'wrenches_stamped'
#include "geometry_msgs/msg/detail/wrench_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rviz_legged_msgs__msg__WrenchesStamped __attribute__((deprecated))
#else
# define DEPRECATED__rviz_legged_msgs__msg__WrenchesStamped __declspec(deprecated)
#endif

namespace rviz_legged_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WrenchesStamped_
{
  using Type = WrenchesStamped_<ContainerAllocator>;

  explicit WrenchesStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit WrenchesStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _wrenches_stamped_type =
    std::vector<geometry_msgs::msg::WrenchStamped_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::WrenchStamped_<ContainerAllocator>>>;
  _wrenches_stamped_type wrenches_stamped;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__wrenches_stamped(
    const std::vector<geometry_msgs::msg::WrenchStamped_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::WrenchStamped_<ContainerAllocator>>> & _arg)
  {
    this->wrenches_stamped = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rviz_legged_msgs::msg::WrenchesStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const rviz_legged_msgs::msg::WrenchesStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rviz_legged_msgs::msg::WrenchesStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rviz_legged_msgs::msg::WrenchesStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rviz_legged_msgs::msg::WrenchesStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rviz_legged_msgs::msg::WrenchesStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rviz_legged_msgs::msg::WrenchesStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rviz_legged_msgs::msg::WrenchesStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rviz_legged_msgs::msg::WrenchesStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rviz_legged_msgs::msg::WrenchesStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rviz_legged_msgs__msg__WrenchesStamped
    std::shared_ptr<rviz_legged_msgs::msg::WrenchesStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rviz_legged_msgs__msg__WrenchesStamped
    std::shared_ptr<rviz_legged_msgs::msg::WrenchesStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WrenchesStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->wrenches_stamped != other.wrenches_stamped) {
      return false;
    }
    return true;
  }
  bool operator!=(const WrenchesStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WrenchesStamped_

// alias to use template instance with default allocator
using WrenchesStamped =
  rviz_legged_msgs::msg::WrenchesStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rviz_legged_msgs

#endif  // RVIZ_LEGGED_MSGS__MSG__DETAIL__WRENCHES_STAMPED__STRUCT_HPP_