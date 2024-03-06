// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from generalized_pose_msgs:msg/GeneralizedPoseWithTime.idl
// generated code does not contain a copyright notice
#include "generalized_pose_msgs/msg/detail/generalized_pose_with_time__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `generalized_pose`
#include "generalized_pose_msgs/msg/detail/generalized_pose__functions.h"

bool
generalized_pose_msgs__msg__GeneralizedPoseWithTime__init(generalized_pose_msgs__msg__GeneralizedPoseWithTime * msg)
{
  if (!msg) {
    return false;
  }
  // generalized_pose
  if (!generalized_pose_msgs__msg__GeneralizedPose__init(&msg->generalized_pose)) {
    generalized_pose_msgs__msg__GeneralizedPoseWithTime__fini(msg);
    return false;
  }
  // time
  return true;
}

void
generalized_pose_msgs__msg__GeneralizedPoseWithTime__fini(generalized_pose_msgs__msg__GeneralizedPoseWithTime * msg)
{
  if (!msg) {
    return;
  }
  // generalized_pose
  generalized_pose_msgs__msg__GeneralizedPose__fini(&msg->generalized_pose);
  // time
}

bool
generalized_pose_msgs__msg__GeneralizedPoseWithTime__are_equal(const generalized_pose_msgs__msg__GeneralizedPoseWithTime * lhs, const generalized_pose_msgs__msg__GeneralizedPoseWithTime * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // generalized_pose
  if (!generalized_pose_msgs__msg__GeneralizedPose__are_equal(
      &(lhs->generalized_pose), &(rhs->generalized_pose)))
  {
    return false;
  }
  // time
  if (lhs->time != rhs->time) {
    return false;
  }
  return true;
}

bool
generalized_pose_msgs__msg__GeneralizedPoseWithTime__copy(
  const generalized_pose_msgs__msg__GeneralizedPoseWithTime * input,
  generalized_pose_msgs__msg__GeneralizedPoseWithTime * output)
{
  if (!input || !output) {
    return false;
  }
  // generalized_pose
  if (!generalized_pose_msgs__msg__GeneralizedPose__copy(
      &(input->generalized_pose), &(output->generalized_pose)))
  {
    return false;
  }
  // time
  output->time = input->time;
  return true;
}

generalized_pose_msgs__msg__GeneralizedPoseWithTime *
generalized_pose_msgs__msg__GeneralizedPoseWithTime__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  generalized_pose_msgs__msg__GeneralizedPoseWithTime * msg = (generalized_pose_msgs__msg__GeneralizedPoseWithTime *)allocator.allocate(sizeof(generalized_pose_msgs__msg__GeneralizedPoseWithTime), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(generalized_pose_msgs__msg__GeneralizedPoseWithTime));
  bool success = generalized_pose_msgs__msg__GeneralizedPoseWithTime__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
generalized_pose_msgs__msg__GeneralizedPoseWithTime__destroy(generalized_pose_msgs__msg__GeneralizedPoseWithTime * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    generalized_pose_msgs__msg__GeneralizedPoseWithTime__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence__init(generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  generalized_pose_msgs__msg__GeneralizedPoseWithTime * data = NULL;

  if (size) {
    data = (generalized_pose_msgs__msg__GeneralizedPoseWithTime *)allocator.zero_allocate(size, sizeof(generalized_pose_msgs__msg__GeneralizedPoseWithTime), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = generalized_pose_msgs__msg__GeneralizedPoseWithTime__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        generalized_pose_msgs__msg__GeneralizedPoseWithTime__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence__fini(generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      generalized_pose_msgs__msg__GeneralizedPoseWithTime__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence *
generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence * array = (generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence *)allocator.allocate(sizeof(generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence__destroy(generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence__are_equal(const generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence * lhs, const generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!generalized_pose_msgs__msg__GeneralizedPoseWithTime__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence__copy(
  const generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence * input,
  generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(generalized_pose_msgs__msg__GeneralizedPoseWithTime);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    generalized_pose_msgs__msg__GeneralizedPoseWithTime * data =
      (generalized_pose_msgs__msg__GeneralizedPoseWithTime *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!generalized_pose_msgs__msg__GeneralizedPoseWithTime__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          generalized_pose_msgs__msg__GeneralizedPoseWithTime__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!generalized_pose_msgs__msg__GeneralizedPoseWithTime__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
