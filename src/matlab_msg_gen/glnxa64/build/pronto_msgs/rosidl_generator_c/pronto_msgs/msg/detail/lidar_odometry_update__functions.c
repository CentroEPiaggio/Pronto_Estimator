// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pronto_msgs:msg/LidarOdometryUpdate.idl
// generated code does not contain a copyright notice
#include "pronto_msgs/msg/detail/lidar_odometry_update__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `curr_timestamp`
// Member `prev_timestamp`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `relative_transform`
#include "geometry_msgs/msg/detail/transform__functions.h"

bool
pronto_msgs__msg__LidarOdometryUpdate__init(pronto_msgs__msg__LidarOdometryUpdate * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    pronto_msgs__msg__LidarOdometryUpdate__fini(msg);
    return false;
  }
  // curr_timestamp
  if (!builtin_interfaces__msg__Time__init(&msg->curr_timestamp)) {
    pronto_msgs__msg__LidarOdometryUpdate__fini(msg);
    return false;
  }
  // prev_timestamp
  if (!builtin_interfaces__msg__Time__init(&msg->prev_timestamp)) {
    pronto_msgs__msg__LidarOdometryUpdate__fini(msg);
    return false;
  }
  // relative_transform
  if (!geometry_msgs__msg__Transform__init(&msg->relative_transform)) {
    pronto_msgs__msg__LidarOdometryUpdate__fini(msg);
    return false;
  }
  // covariance
  return true;
}

void
pronto_msgs__msg__LidarOdometryUpdate__fini(pronto_msgs__msg__LidarOdometryUpdate * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // curr_timestamp
  builtin_interfaces__msg__Time__fini(&msg->curr_timestamp);
  // prev_timestamp
  builtin_interfaces__msg__Time__fini(&msg->prev_timestamp);
  // relative_transform
  geometry_msgs__msg__Transform__fini(&msg->relative_transform);
  // covariance
}

pronto_msgs__msg__LidarOdometryUpdate *
pronto_msgs__msg__LidarOdometryUpdate__create()
{
  pronto_msgs__msg__LidarOdometryUpdate * msg = (pronto_msgs__msg__LidarOdometryUpdate *)malloc(sizeof(pronto_msgs__msg__LidarOdometryUpdate));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pronto_msgs__msg__LidarOdometryUpdate));
  bool success = pronto_msgs__msg__LidarOdometryUpdate__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
pronto_msgs__msg__LidarOdometryUpdate__destroy(pronto_msgs__msg__LidarOdometryUpdate * msg)
{
  if (msg) {
    pronto_msgs__msg__LidarOdometryUpdate__fini(msg);
  }
  free(msg);
}


bool
pronto_msgs__msg__LidarOdometryUpdate__Sequence__init(pronto_msgs__msg__LidarOdometryUpdate__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  pronto_msgs__msg__LidarOdometryUpdate * data = NULL;
  if (size) {
    data = (pronto_msgs__msg__LidarOdometryUpdate *)calloc(size, sizeof(pronto_msgs__msg__LidarOdometryUpdate));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pronto_msgs__msg__LidarOdometryUpdate__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pronto_msgs__msg__LidarOdometryUpdate__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
pronto_msgs__msg__LidarOdometryUpdate__Sequence__fini(pronto_msgs__msg__LidarOdometryUpdate__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      pronto_msgs__msg__LidarOdometryUpdate__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

pronto_msgs__msg__LidarOdometryUpdate__Sequence *
pronto_msgs__msg__LidarOdometryUpdate__Sequence__create(size_t size)
{
  pronto_msgs__msg__LidarOdometryUpdate__Sequence * array = (pronto_msgs__msg__LidarOdometryUpdate__Sequence *)malloc(sizeof(pronto_msgs__msg__LidarOdometryUpdate__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = pronto_msgs__msg__LidarOdometryUpdate__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
pronto_msgs__msg__LidarOdometryUpdate__Sequence__destroy(pronto_msgs__msg__LidarOdometryUpdate__Sequence * array)
{
  if (array) {
    pronto_msgs__msg__LidarOdometryUpdate__Sequence__fini(array);
  }
  free(array);
}
