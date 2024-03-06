// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from generalized_pose_msgs:msg/GeneralizedPoseWithTime.idl
// generated code does not contain a copyright notice

#ifndef GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSE_WITH_TIME__STRUCT_HPP_
#define GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSE_WITH_TIME__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'generalized_pose'
#include "generalized_pose_msgs/msg/detail/generalized_pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__generalized_pose_msgs__msg__GeneralizedPoseWithTime __attribute__((deprecated))
#else
# define DEPRECATED__generalized_pose_msgs__msg__GeneralizedPoseWithTime __declspec(deprecated)
#endif

namespace generalized_pose_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GeneralizedPoseWithTime_
{
  using Type = GeneralizedPoseWithTime_<ContainerAllocator>;

  explicit GeneralizedPoseWithTime_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : generalized_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time = 0.0;
    }
  }

  explicit GeneralizedPoseWithTime_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : generalized_pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time = 0.0;
    }
  }

  // field types and members
  using _generalized_pose_type =
    generalized_pose_msgs::msg::GeneralizedPose_<ContainerAllocator>;
  _generalized_pose_type generalized_pose;
  using _time_type =
    double;
  _time_type time;

  // setters for named parameter idiom
  Type & set__generalized_pose(
    const generalized_pose_msgs::msg::GeneralizedPose_<ContainerAllocator> & _arg)
  {
    this->generalized_pose = _arg;
    return *this;
  }
  Type & set__time(
    const double & _arg)
  {
    this->time = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    generalized_pose_msgs::msg::GeneralizedPoseWithTime_<ContainerAllocator> *;
  using ConstRawPtr =
    const generalized_pose_msgs::msg::GeneralizedPoseWithTime_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<generalized_pose_msgs::msg::GeneralizedPoseWithTime_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<generalized_pose_msgs::msg::GeneralizedPoseWithTime_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      generalized_pose_msgs::msg::GeneralizedPoseWithTime_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<generalized_pose_msgs::msg::GeneralizedPoseWithTime_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      generalized_pose_msgs::msg::GeneralizedPoseWithTime_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<generalized_pose_msgs::msg::GeneralizedPoseWithTime_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<generalized_pose_msgs::msg::GeneralizedPoseWithTime_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<generalized_pose_msgs::msg::GeneralizedPoseWithTime_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__generalized_pose_msgs__msg__GeneralizedPoseWithTime
    std::shared_ptr<generalized_pose_msgs::msg::GeneralizedPoseWithTime_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__generalized_pose_msgs__msg__GeneralizedPoseWithTime
    std::shared_ptr<generalized_pose_msgs::msg::GeneralizedPoseWithTime_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GeneralizedPoseWithTime_ & other) const
  {
    if (this->generalized_pose != other.generalized_pose) {
      return false;
    }
    if (this->time != other.time) {
      return false;
    }
    return true;
  }
  bool operator!=(const GeneralizedPoseWithTime_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GeneralizedPoseWithTime_

// alias to use template instance with default allocator
using GeneralizedPoseWithTime =
  generalized_pose_msgs::msg::GeneralizedPoseWithTime_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace generalized_pose_msgs

#endif  // GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSE_WITH_TIME__STRUCT_HPP_
