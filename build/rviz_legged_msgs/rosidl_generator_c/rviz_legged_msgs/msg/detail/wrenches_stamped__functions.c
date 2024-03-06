// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rviz_legged_msgs:msg/WrenchesStamped.idl
// generated code does not contain a copyright notice
#include "rviz_legged_msgs/msg/detail/wrenches_stamped__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `wrenches_stamped`
#include "geometry_msgs/msg/detail/wrench_stamped__functions.h"

bool
rviz_legged_msgs__msg__WrenchesStamped__init(rviz_legged_msgs__msg__WrenchesStamped * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    rviz_legged_msgs__msg__WrenchesStamped__fini(msg);
    return false;
  }
  // wrenches_stamped
  if (!geometry_msgs__msg__WrenchStamped__Sequence__init(&msg->wrenches_stamped, 0)) {
    rviz_legged_msgs__msg__WrenchesStamped__fini(msg);
    return false;
  }
  return true;
}

void
rviz_legged_msgs__msg__WrenchesStamped__fini(rviz_legged_msgs__msg__WrenchesStamped * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // wrenches_stamped
  geometry_msgs__msg__WrenchStamped__Sequence__fini(&msg->wrenches_stamped);
}

bool
rviz_legged_msgs__msg__WrenchesStamped__are_equal(const rviz_legged_msgs__msg__WrenchesStamped * lhs, const rviz_legged_msgs__msg__WrenchesStamped * rhs)
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
  // wrenches_stamped
  if (!geometry_msgs__msg__WrenchStamped__Sequence__are_equal(
      &(lhs->wrenches_stamped), &(rhs->wrenches_stamped)))
  {
    return false;
  }
  return true;
}

bool
rviz_legged_msgs__msg__WrenchesStamped__copy(
  const rviz_legged_msgs__msg__WrenchesStamped * input,
  rviz_legged_msgs__msg__WrenchesStamped * output)
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
  // wrenches_stamped
  if (!geometry_msgs__msg__WrenchStamped__Sequence__copy(
      &(input->wrenches_stamped), &(output->wrenches_stamped)))
  {
    return false;
  }
  return true;
}

rviz_legged_msgs__msg__WrenchesStamped *
rviz_legged_msgs__msg__WrenchesStamped__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rviz_legged_msgs__msg__WrenchesStamped * msg = (rviz_legged_msgs__msg__WrenchesStamped *)allocator.allocate(sizeof(rviz_legged_msgs__msg__WrenchesStamped), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rviz_legged_msgs__msg__WrenchesStamped));
  bool success = rviz_legged_msgs__msg__WrenchesStamped__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rviz_legged_msgs__msg__WrenchesStamped__destroy(rviz_legged_msgs__msg__WrenchesStamped * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rviz_legged_msgs__msg__WrenchesStamped__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rviz_legged_msgs__msg__WrenchesStamped__Sequence__init(rviz_legged_msgs__msg__WrenchesStamped__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rviz_legged_msgs__msg__WrenchesStamped * data = NULL;

  if (size) {
    data = (rviz_legged_msgs__msg__WrenchesStamped *)allocator.zero_allocate(size, sizeof(rviz_legged_msgs__msg__WrenchesStamped), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rviz_legged_msgs__msg__WrenchesStamped__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rviz_legged_msgs__msg__WrenchesStamped__fini(&data[i - 1]);
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
rviz_legged_msgs__msg__WrenchesStamped__Sequence__fini(rviz_legged_msgs__msg__WrenchesStamped__Sequence * array)
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
      rviz_legged_msgs__msg__WrenchesStamped__fini(&array->data[i]);
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

rviz_legged_msgs__msg__WrenchesStamped__Sequence *
rviz_legged_msgs__msg__WrenchesStamped__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rviz_legged_msgs__msg__WrenchesStamped__Sequence * array = (rviz_legged_msgs__msg__WrenchesStamped__Sequence *)allocator.allocate(sizeof(rviz_legged_msgs__msg__WrenchesStamped__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rviz_legged_msgs__msg__WrenchesStamped__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rviz_legged_msgs__msg__WrenchesStamped__Sequence__destroy(rviz_legged_msgs__msg__WrenchesStamped__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rviz_legged_msgs__msg__WrenchesStamped__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rviz_legged_msgs__msg__WrenchesStamped__Sequence__are_equal(const rviz_legged_msgs__msg__WrenchesStamped__Sequence * lhs, const rviz_legged_msgs__msg__WrenchesStamped__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rviz_legged_msgs__msg__WrenchesStamped__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rviz_legged_msgs__msg__WrenchesStamped__Sequence__copy(
  const rviz_legged_msgs__msg__WrenchesStamped__Sequence * input,
  rviz_legged_msgs__msg__WrenchesStamped__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rviz_legged_msgs__msg__WrenchesStamped);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rviz_legged_msgs__msg__WrenchesStamped * data =
      (rviz_legged_msgs__msg__WrenchesStamped *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rviz_legged_msgs__msg__WrenchesStamped__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rviz_legged_msgs__msg__WrenchesStamped__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rviz_legged_msgs__msg__WrenchesStamped__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
