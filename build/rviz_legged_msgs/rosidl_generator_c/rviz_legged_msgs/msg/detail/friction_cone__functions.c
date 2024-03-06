// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rviz_legged_msgs:msg/FrictionCone.idl
// generated code does not contain a copyright notice
#include "rviz_legged_msgs/msg/detail/friction_cone__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `normal_direction`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
rviz_legged_msgs__msg__FrictionCone__init(rviz_legged_msgs__msg__FrictionCone * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    rviz_legged_msgs__msg__FrictionCone__fini(msg);
    return false;
  }
  // normal_direction
  if (!geometry_msgs__msg__Vector3__init(&msg->normal_direction)) {
    rviz_legged_msgs__msg__FrictionCone__fini(msg);
    return false;
  }
  // friction_coefficient
  return true;
}

void
rviz_legged_msgs__msg__FrictionCone__fini(rviz_legged_msgs__msg__FrictionCone * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // normal_direction
  geometry_msgs__msg__Vector3__fini(&msg->normal_direction);
  // friction_coefficient
}

bool
rviz_legged_msgs__msg__FrictionCone__are_equal(const rviz_legged_msgs__msg__FrictionCone * lhs, const rviz_legged_msgs__msg__FrictionCone * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // normal_direction
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->normal_direction), &(rhs->normal_direction)))
  {
    return false;
  }
  // friction_coefficient
  if (lhs->friction_coefficient != rhs->friction_coefficient) {
    return false;
  }
  return true;
}

bool
rviz_legged_msgs__msg__FrictionCone__copy(
  const rviz_legged_msgs__msg__FrictionCone * input,
  rviz_legged_msgs__msg__FrictionCone * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // normal_direction
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->normal_direction), &(output->normal_direction)))
  {
    return false;
  }
  // friction_coefficient
  output->friction_coefficient = input->friction_coefficient;
  return true;
}

rviz_legged_msgs__msg__FrictionCone *
rviz_legged_msgs__msg__FrictionCone__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rviz_legged_msgs__msg__FrictionCone * msg = (rviz_legged_msgs__msg__FrictionCone *)allocator.allocate(sizeof(rviz_legged_msgs__msg__FrictionCone), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rviz_legged_msgs__msg__FrictionCone));
  bool success = rviz_legged_msgs__msg__FrictionCone__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rviz_legged_msgs__msg__FrictionCone__destroy(rviz_legged_msgs__msg__FrictionCone * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rviz_legged_msgs__msg__FrictionCone__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rviz_legged_msgs__msg__FrictionCone__Sequence__init(rviz_legged_msgs__msg__FrictionCone__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rviz_legged_msgs__msg__FrictionCone * data = NULL;

  if (size) {
    data = (rviz_legged_msgs__msg__FrictionCone *)allocator.zero_allocate(size, sizeof(rviz_legged_msgs__msg__FrictionCone), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rviz_legged_msgs__msg__FrictionCone__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rviz_legged_msgs__msg__FrictionCone__fini(&data[i - 1]);
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
rviz_legged_msgs__msg__FrictionCone__Sequence__fini(rviz_legged_msgs__msg__FrictionCone__Sequence * array)
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
      rviz_legged_msgs__msg__FrictionCone__fini(&array->data[i]);
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

rviz_legged_msgs__msg__FrictionCone__Sequence *
rviz_legged_msgs__msg__FrictionCone__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rviz_legged_msgs__msg__FrictionCone__Sequence * array = (rviz_legged_msgs__msg__FrictionCone__Sequence *)allocator.allocate(sizeof(rviz_legged_msgs__msg__FrictionCone__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rviz_legged_msgs__msg__FrictionCone__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rviz_legged_msgs__msg__FrictionCone__Sequence__destroy(rviz_legged_msgs__msg__FrictionCone__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rviz_legged_msgs__msg__FrictionCone__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rviz_legged_msgs__msg__FrictionCone__Sequence__are_equal(const rviz_legged_msgs__msg__FrictionCone__Sequence * lhs, const rviz_legged_msgs__msg__FrictionCone__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rviz_legged_msgs__msg__FrictionCone__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rviz_legged_msgs__msg__FrictionCone__Sequence__copy(
  const rviz_legged_msgs__msg__FrictionCone__Sequence * input,
  rviz_legged_msgs__msg__FrictionCone__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rviz_legged_msgs__msg__FrictionCone);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rviz_legged_msgs__msg__FrictionCone * data =
      (rviz_legged_msgs__msg__FrictionCone *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rviz_legged_msgs__msg__FrictionCone__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rviz_legged_msgs__msg__FrictionCone__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rviz_legged_msgs__msg__FrictionCone__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
