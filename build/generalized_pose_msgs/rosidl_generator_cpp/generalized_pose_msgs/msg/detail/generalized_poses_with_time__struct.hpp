// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from generalized_pose_msgs:msg/GeneralizedPosesWithTime.idl
// generated code does not contain a copyright notice

#ifndef GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSES_WITH_TIME__STRUCT_HPP_
#define GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSES_WITH_TIME__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'generalized_poses_with_time'
#include "generalized_pose_msgs/msg/detail/generalized_pose_with_time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__generalized_pose_msgs__msg__GeneralizedPosesWithTime __attribute__((deprecated))
#else
# define DEPRECATED__generalized_pose_msgs__msg__GeneralizedPosesWithTime __declspec(deprecated)
#endif

namespace generalized_pose_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GeneralizedPosesWithTime_
{
  using Type = GeneralizedPosesWithTime_<ContainerAllocator>;

  explicit GeneralizedPosesWithTime_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit GeneralizedPosesWithTime_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _generalized_poses_with_time_type =
    std::vector<generalized_pose_msgs::msg::GeneralizedPoseWithTime_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<generalized_pose_msgs::msg::GeneralizedPoseWithTime_<ContainerAllocator>>>;
  _generalized_poses_with_time_type generalized_poses_with_time;

  // setters for named parameter idiom
  Type & set__generalized_poses_with_time(
    const std::vector<generalized_pose_msgs::msg::GeneralizedPoseWithTime_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<generalized_pose_msgs::msg::GeneralizedPoseWithTime_<ContainerAllocator>>> & _arg)
  {
    this->generalized_poses_with_time = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    generalized_pose_msgs::msg::GeneralizedPosesWithTime_<ContainerAllocator> *;
  using ConstRawPtr =
    const generalized_pose_msgs::msg::GeneralizedPosesWithTime_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<generalized_pose_msgs::msg::GeneralizedPosesWithTime_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<generalized_pose_msgs::msg::GeneralizedPosesWithTime_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      generalized_pose_msgs::msg::GeneralizedPosesWithTime_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<generalized_pose_msgs::msg::GeneralizedPosesWithTime_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      generalized_pose_msgs::msg::GeneralizedPosesWithTime_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<generalized_pose_msgs::msg::GeneralizedPosesWithTime_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<generalized_pose_msgs::msg::GeneralizedPosesWithTime_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<generalized_pose_msgs::msg::GeneralizedPosesWithTime_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__generalized_pose_msgs__msg__GeneralizedPosesWithTime
    std::shared_ptr<generalized_pose_msgs::msg::GeneralizedPosesWithTime_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__generalized_pose_msgs__msg__GeneralizedPosesWithTime
    std::shared_ptr<generalized_pose_msgs::msg::GeneralizedPosesWithTime_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GeneralizedPosesWithTime_ & other) const
  {
    if (this->generalized_poses_with_time != other.generalized_poses_with_time) {
      return false;
    }
    return true;
  }
  bool operator!=(const GeneralizedPosesWithTime_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GeneralizedPosesWithTime_

// alias to use template instance with default allocator
using GeneralizedPosesWithTime =
  generalized_pose_msgs::msg::GeneralizedPosesWithTime_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace generalized_pose_msgs

#endif  // GENERALIZED_POSE_MSGS__MSG__DETAIL__GENERALIZED_POSES_WITH_TIME__STRUCT_HPP_
