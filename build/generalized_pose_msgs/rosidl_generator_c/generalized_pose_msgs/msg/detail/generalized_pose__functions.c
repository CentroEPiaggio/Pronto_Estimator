// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from generalized_pose_msgs:msg/GeneralizedPose.idl
// generated code does not contain a copyright notice
#include "generalized_pose_msgs/msg/detail/generalized_pose__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `base_acc`
// Member `base_vel`
// Member `base_pos`
// Member `base_angvel`
#include "geometry_msgs/msg/detail/vector3__functions.h"
// Member `base_quat`
#include "geometry_msgs/msg/detail/quaternion__functions.h"
// Member `feet_acc`
// Member `feet_vel`
// Member `feet_pos`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `contact_feet`
#include "rosidl_runtime_c/string_functions.h"

bool
generalized_pose_msgs__msg__GeneralizedPose__init(generalized_pose_msgs__msg__GeneralizedPose * msg)
{
  if (!msg) {
    return false;
  }
  // base_acc
  if (!geometry_msgs__msg__Vector3__init(&msg->base_acc)) {
    generalized_pose_msgs__msg__GeneralizedPose__fini(msg);
    return false;
  }
  // base_vel
  if (!geometry_msgs__msg__Vector3__init(&msg->base_vel)) {
    generalized_pose_msgs__msg__GeneralizedPose__fini(msg);
    return false;
  }
  // base_pos
  if (!geometry_msgs__msg__Vector3__init(&msg->base_pos)) {
    generalized_pose_msgs__msg__GeneralizedPose__fini(msg);
    return false;
  }
  // base_angvel
  if (!geometry_msgs__msg__Vector3__init(&msg->base_angvel)) {
    generalized_pose_msgs__msg__GeneralizedPose__fini(msg);
    return false;
  }
  // base_quat
  if (!geometry_msgs__msg__Quaternion__init(&msg->base_quat)) {
    generalized_pose_msgs__msg__GeneralizedPose__fini(msg);
    return false;
  }
  // feet_acc
  if (!rosidl_runtime_c__double__Sequence__init(&msg->feet_acc, 0)) {
    generalized_pose_msgs__msg__GeneralizedPose__fini(msg);
    return false;
  }
  // feet_vel
  if (!rosidl_runtime_c__double__Sequence__init(&msg->feet_vel, 0)) {
    generalized_pose_msgs__msg__GeneralizedPose__fini(msg);
    return false;
  }
  // feet_pos
  if (!rosidl_runtime_c__double__Sequence__init(&msg->feet_pos, 0)) {
    generalized_pose_msgs__msg__GeneralizedPose__fini(msg);
    return false;
  }
  // contact_feet
  if (!rosidl_runtime_c__String__Sequence__init(&msg->contact_feet, 0)) {
    generalized_pose_msgs__msg__GeneralizedPose__fini(msg);
    return false;
  }
  return true;
}

void
generalized_pose_msgs__msg__GeneralizedPose__fini(generalized_pose_msgs__msg__GeneralizedPose * msg)
{
  if (!msg) {
    return;
  }
  // base_acc
  geometry_msgs__msg__Vector3__fini(&msg->base_acc);
  // base_vel
  geometry_msgs__msg__Vector3__fini(&msg->base_vel);
  // base_pos
  geometry_msgs__msg__Vector3__fini(&msg->base_pos);
  // base_angvel
  geometry_msgs__msg__Vector3__fini(&msg->base_angvel);
  // base_quat
  geometry_msgs__msg__Quaternion__fini(&msg->base_quat);
  // feet_acc
  rosidl_runtime_c__double__Sequence__fini(&msg->feet_acc);
  // feet_vel
  rosidl_runtime_c__double__Sequence__fini(&msg->feet_vel);
  // feet_pos
  rosidl_runtime_c__double__Sequence__fini(&msg->feet_pos);
  // contact_feet
  rosidl_runtime_c__String__Sequence__fini(&msg->contact_feet);
}

bool
generalized_pose_msgs__msg__GeneralizedPose__are_equal(const generalized_pose_msgs__msg__GeneralizedPose * lhs, const generalized_pose_msgs__msg__GeneralizedPose * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // base_acc
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->base_acc), &(rhs->base_acc)))
  {
    return false;
  }
  // base_vel
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->base_vel), &(rhs->base_vel)))
  {
    return false;
  }
  // base_pos
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->base_pos), &(rhs->base_pos)))
  {
    return false;
  }
  // base_angvel
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->base_angvel), &(rhs->base_angvel)))
  {
    return false;
  }
  // base_quat
  if (!geometry_msgs__msg__Quaternion__are_equal(
      &(lhs->base_quat), &(rhs->base_quat)))
  {
    return false;
  }
  // feet_acc
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->feet_acc), &(rhs->feet_acc)))
  {
    return false;
  }
  // feet_vel
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->feet_vel), &(rhs->feet_vel)))
  {
    return false;
  }
  // feet_pos
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->feet_pos), &(rhs->feet_pos)))
  {
    return false;
  }
  // contact_feet
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->contact_feet), &(rhs->contact_feet)))
  {
    return false;
  }
  return true;
}

bool
generalized_pose_msgs__msg__GeneralizedPose__copy(
  const generalized_pose_msgs__msg__GeneralizedPose * input,
  generalized_pose_msgs__msg__GeneralizedPose * output)
{
  if (!input || !output) {
    return false;
  }
  // base_acc
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->base_acc), &(output->base_acc)))
  {
    return false;
  }
  // base_vel
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->base_vel), &(output->base_vel)))
  {
    return false;
  }
  // base_pos
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->base_pos), &(output->base_pos)))
  {
    return false;
  }
  // base_angvel
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->base_angvel), &(output->base_angvel)))
  {
    return false;
  }
  // base_quat
  if (!geometry_msgs__msg__Quaternion__copy(
      &(input->base_quat), &(output->base_quat)))
  {
    return false;
  }
  // feet_acc
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->feet_acc), &(output->feet_acc)))
  {
    return false;
  }
  // feet_vel
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->feet_vel), &(output->feet_vel)))
  {
    return false;
  }
  // feet_pos
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->feet_pos), &(output->feet_pos)))
  {
    return false;
  }
  // contact_feet
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->contact_feet), &(output->contact_feet)))
  {
    return false;
  }
  return true;
}

generalized_pose_msgs__msg__GeneralizedPose *
generalized_pose_msgs__msg__GeneralizedPose__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  generalized_pose_msgs__msg__GeneralizedPose * msg = (generalized_pose_msgs__msg__GeneralizedPose *)allocator.allocate(sizeof(generalized_pose_msgs__msg__GeneralizedPose), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(generalized_pose_msgs__msg__GeneralizedPose));
  bool success = generalized_pose_msgs__msg__GeneralizedPose__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
generalized_pose_msgs__msg__GeneralizedPose__destroy(generalized_pose_msgs__msg__GeneralizedPose * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    generalized_pose_msgs__msg__GeneralizedPose__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
generalized_pose_msgs__msg__GeneralizedPose__Sequence__init(generalized_pose_msgs__msg__GeneralizedPose__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  generalized_pose_msgs__msg__GeneralizedPose * data = NULL;

  if (size) {
    data = (generalized_pose_msgs__msg__GeneralizedPose *)allocator.zero_allocate(size, sizeof(generalized_pose_msgs__msg__GeneralizedPose), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = generalized_pose_msgs__msg__GeneralizedPose__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        generalized_pose_msgs__msg__GeneralizedPose__fini(&data[i - 1]);
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
generalized_pose_msgs__msg__GeneralizedPose__Sequence__fini(generalized_pose_msgs__msg__GeneralizedPose__Sequence * array)
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
      generalized_pose_msgs__msg__GeneralizedPose__fini(&array->data[i]);
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

generalized_pose_msgs__msg__GeneralizedPose__Sequence *
generalized_pose_msgs__msg__GeneralizedPose__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  generalized_pose_msgs__msg__GeneralizedPose__Sequence * array = (generalized_pose_msgs__msg__GeneralizedPose__Sequence *)allocator.allocate(sizeof(generalized_pose_msgs__msg__GeneralizedPose__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = generalized_pose_msgs__msg__GeneralizedPose__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
generalized_pose_msgs__msg__GeneralizedPose__Sequence__destroy(generalized_pose_msgs__msg__GeneralizedPose__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    generalized_pose_msgs__msg__GeneralizedPose__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
generalized_pose_msgs__msg__GeneralizedPose__Sequence__are_equal(const generalized_pose_msgs__msg__GeneralizedPose__Sequence * lhs, const generalized_pose_msgs__msg__GeneralizedPose__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!generalized_pose_msgs__msg__GeneralizedPose__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
generalized_pose_msgs__msg__GeneralizedPose__Sequence__copy(
  const generalized_pose_msgs__msg__GeneralizedPose__Sequence * input,
  generalized_pose_msgs__msg__GeneralizedPose__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(generalized_pose_msgs__msg__GeneralizedPose);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    generalized_pose_msgs__msg__GeneralizedPose * data =
      (generalized_pose_msgs__msg__GeneralizedPose *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!generalized_pose_msgs__msg__GeneralizedPose__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          generalized_pose_msgs__msg__GeneralizedPose__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!generalized_pose_msgs__msg__GeneralizedPose__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
