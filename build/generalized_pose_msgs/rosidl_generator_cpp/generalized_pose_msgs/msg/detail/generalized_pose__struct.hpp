// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from generalized_pose_msgs:msg/GeneralizedPose.idl
// generated code does not contain a copyright notice

#ifndef GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSE__STRUCT_HPP_
#define GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'base_acc'
// Member 'base_vel'
// Member 'base_pos'
// Member 'base_angvel'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"
// Member 'base_quat'
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__generalized_pose_msgs__msg__GeneralizedPose __attribute__((deprecated))
#else
# define DEPRECATED__generalized_pose_msgs__msg__GeneralizedPose __declspec(deprecated)
#endif

namespace generalized_pose_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GeneralizedPose_
{
  using Type = GeneralizedPose_<ContainerAllocator>;

  explicit GeneralizedPose_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : base_acc(_init),
    base_vel(_init),
    base_pos(_init),
    base_angvel(_init),
    base_quat(_init)
  {
    (void)_init;
  }

  explicit GeneralizedPose_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : base_acc(_alloc, _init),
    base_vel(_alloc, _init),
    base_pos(_alloc, _init),
    base_angvel(_alloc, _init),
    base_quat(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _base_acc_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _base_acc_type base_acc;
  using _base_vel_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _base_vel_type base_vel;
  using _base_pos_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _base_pos_type base_pos;
  using _base_angvel_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _base_angvel_type base_angvel;
  using _base_quat_type =
    geometry_msgs::msg::Quaternion_<ContainerAllocator>;
  _base_quat_type base_quat;
  using _feet_acc_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _feet_acc_type feet_acc;
  using _feet_vel_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _feet_vel_type feet_vel;
  using _feet_pos_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _feet_pos_type feet_pos;
  using _contact_feet_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _contact_feet_type contact_feet;

  // setters for named parameter idiom
  Type & set__base_acc(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->base_acc = _arg;
    return *this;
  }
  Type & set__base_vel(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->base_vel = _arg;
    return *this;
  }
  Type & set__base_pos(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->base_pos = _arg;
    return *this;
  }
  Type & set__base_angvel(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->base_angvel = _arg;
    return *this;
  }
  Type & set__base_quat(
    const geometry_msgs::msg::Quaternion_<ContainerAllocator> & _arg)
  {
    this->base_quat = _arg;
    return *this;
  }
  Type & set__feet_acc(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->feet_acc = _arg;
    return *this;
  }
  Type & set__feet_vel(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->feet_vel = _arg;
    return *this;
  }
  Type & set__feet_pos(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->feet_pos = _arg;
    return *this;
  }
  Type & set__contact_feet(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->contact_feet = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    generalized_pose_msgs::msg::GeneralizedPose_<ContainerAllocator> *;
  using ConstRawPtr =
    const generalized_pose_msgs::msg::GeneralizedPose_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<generalized_pose_msgs::msg::GeneralizedPose_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<generalized_pose_msgs::msg::GeneralizedPose_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      generalized_pose_msgs::msg::GeneralizedPose_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<generalized_pose_msgs::msg::GeneralizedPose_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      generalized_pose_msgs::msg::GeneralizedPose_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<generalized_pose_msgs::msg::GeneralizedPose_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<generalized_pose_msgs::msg::GeneralizedPose_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<generalized_pose_msgs::msg::GeneralizedPose_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__generalized_pose_msgs__msg__GeneralizedPose
    std::shared_ptr<generalized_pose_msgs::msg::GeneralizedPose_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__generalized_pose_msgs__msg__GeneralizedPose
    std::shared_ptr<generalized_pose_msgs::msg::GeneralizedPose_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GeneralizedPose_ & other) const
  {
    if (this->base_acc != other.base_acc) {
      return false;
    }
    if (this->base_vel != other.base_vel) {
      return false;
    }
    if (this->base_pos != other.base_pos) {
      return false;
    }
    if (this->base_angvel != other.base_angvel) {
      return false;
    }
    if (this->base_quat != other.base_quat) {
      return false;
    }
    if (this->feet_acc != other.feet_acc) {
      return false;
    }
    if (this->feet_vel != other.feet_vel) {
      return false;
    }
    if (this->feet_pos != other.feet_pos) {
      return false;
    }
    if (this->contact_feet != other.contact_feet) {
      return false;
    }
    return true;
  }
  bool operator!=(const GeneralizedPose_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GeneralizedPose_

// alias to use template instance with default allocator
using GeneralizedPose =
  generalized_pose_msgs::msg::GeneralizedPose_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace generalized_pose_msgs

#endif  // GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSE__STRUCT_HPP_
