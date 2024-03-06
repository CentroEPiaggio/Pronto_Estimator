// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pronto_msgs:msg/LidarOdometryUpdate.idl
// generated code does not contain a copyright notice

#ifndef PRONTO_MSGS__MSG__DETAIL__LIDAR_ODOMETRY_UPDATE__TRAITS_HPP_
#define PRONTO_MSGS__MSG__DETAIL__LIDAR_ODOMETRY_UPDATE__TRAITS_HPP_

#include "pronto_msgs/msg/detail/lidar_odometry_update__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'curr_timestamp'
// Member 'prev_timestamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"
// Member 'relative_transform'
#include "geometry_msgs/msg/detail/transform__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<pronto_msgs::msg::LidarOdometryUpdate>()
{
  return "pronto_msgs::msg::LidarOdometryUpdate";
}

template<>
inline const char * name<pronto_msgs::msg::LidarOdometryUpdate>()
{
  return "pronto_msgs/msg/LidarOdometryUpdate";
}

template<>
struct has_fixed_size<pronto_msgs::msg::LidarOdometryUpdate>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value && has_fixed_size<geometry_msgs::msg::Transform>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<pronto_msgs::msg::LidarOdometryUpdate>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value && has_bounded_size<geometry_msgs::msg::Transform>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<pronto_msgs::msg::LidarOdometryUpdate>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PRONTO_MSGS__MSG__DETAIL__LIDAR_ODOMETRY_UPDATE__TRAITS_HPP_
