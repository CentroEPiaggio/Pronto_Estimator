// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pronto_msgs:msg/JointStateWithAcceleration.idl
// generated code does not contain a copyright notice
#include "pronto_msgs/msg/detail/joint_state_with_acceleration__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `position`
// Member `velocity`
// Member `acceleration`
// Member `effort`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
pronto_msgs__msg__JointStateWithAcceleration__init(pronto_msgs__msg__JointStateWithAcceleration * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    pronto_msgs__msg__JointStateWithAcceleration__fini(msg);
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__Sequence__init(&msg->name, 0)) {
    pronto_msgs__msg__JointStateWithAcceleration__fini(msg);
    return false;
  }
  // position
  if (!rosidl_runtime_c__double__Sequence__init(&msg->position, 0)) {
    pronto_msgs__msg__JointStateWithAcceleration__fini(msg);
    return false;
  }
  // velocity
  if (!rosidl_runtime_c__double__Sequence__init(&msg->velocity, 0)) {
    pronto_msgs__msg__JointStateWithAcceleration__fini(msg);
    return false;
  }
  // acceleration
  if (!rosidl_runtime_c__double__Sequence__init(&msg->acceleration, 0)) {
    pronto_msgs__msg__JointStateWithAcceleration__fini(msg);
    return false;
  }
  // effort
  if (!rosidl_runtime_c__double__Sequence__init(&msg->effort, 0)) {
    pronto_msgs__msg__JointStateWithAcceleration__fini(msg);
    return false;
  }
  return true;
}

void
pronto_msgs__msg__JointStateWithAcceleration__fini(pronto_msgs__msg__JointStateWithAcceleration * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // name
  rosidl_runtime_c__String__Sequence__fini(&msg->name);
  // position
  rosidl_runtime_c__double__Sequence__fini(&msg->position);
  // velocity
  rosidl_runtime_c__double__Sequence__fini(&msg->velocity);
  // acceleration
  rosidl_runtime_c__double__Sequence__fini(&msg->acceleration);
  // effort
  rosidl_runtime_c__double__Sequence__fini(&msg->effort);
}

bool
pronto_msgs__msg__JointStateWithAcceleration__are_equal(const pronto_msgs__msg__JointStateWithAcceleration * lhs, const pronto_msgs__msg__JointStateWithAcceleration * rhs)
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
  // name
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // position
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // velocity
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->velocity), &(rhs->velocity)))
  {
    return false;
  }
  // acceleration
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->acceleration), &(rhs->acceleration)))
  {
    return false;
  }
  // effort
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->effort), &(rhs->effort)))
  {
    return false;
  }
  return true;
}

bool
pronto_msgs__msg__JointStateWithAcceleration__copy(
  const pronto_msgs__msg__JointStateWithAcceleration * input,
  pronto_msgs__msg__JointStateWithAcceleration * output)
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
  // name
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // position
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // velocity
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->velocity), &(output->velocity)))
  {
    return false;
  }
  // acceleration
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->acceleration), &(output->acceleration)))
  {
    return false;
  }
  // effort
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->effort), &(output->effort)))
  {
    return false;
  }
  return true;
}

pronto_msgs__msg__JointStateWithAcceleration *
pronto_msgs__msg__JointStateWithAcceleration__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pronto_msgs__msg__JointStateWithAcceleration * msg = (pronto_msgs__msg__JointStateWithAcceleration *)allocator.allocate(sizeof(pronto_msgs__msg__JointStateWithAcceleration), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pronto_msgs__msg__JointStateWithAcceleration));
  bool success = pronto_msgs__msg__JointStateWithAcceleration__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pronto_msgs__msg__JointStateWithAcceleration__destroy(pronto_msgs__msg__JointStateWithAcceleration * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pronto_msgs__msg__JointStateWithAcceleration__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pronto_msgs__msg__JointStateWithAcceleration__Sequence__init(pronto_msgs__msg__JointStateWithAcceleration__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pronto_msgs__msg__JointStateWithAcceleration * data = NULL;

  if (size) {
    data = (pronto_msgs__msg__JointStateWithAcceleration *)allocator.zero_allocate(size, sizeof(pronto_msgs__msg__JointStateWithAcceleration), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pronto_msgs__msg__JointStateWithAcceleration__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pronto_msgs__msg__JointStateWithAcceleration__fini(&data[i - 1]);
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
pronto_msgs__msg__JointStateWithAcceleration__Sequence__fini(pronto_msgs__msg__JointStateWithAcceleration__Sequence * array)
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
      pronto_msgs__msg__JointStateWithAcceleration__fini(&array->data[i]);
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

pronto_msgs__msg__JointStateWithAcceleration__Sequence *
pronto_msgs__msg__JointStateWithAcceleration__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pronto_msgs__msg__JointStateWithAcceleration__Sequence * array = (pronto_msgs__msg__JointStateWithAcceleration__Sequence *)allocator.allocate(sizeof(pronto_msgs__msg__JointStateWithAcceleration__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pronto_msgs__msg__JointStateWithAcceleration__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pronto_msgs__msg__JointStateWithAcceleration__Sequence__destroy(pronto_msgs__msg__JointStateWithAcceleration__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pronto_msgs__msg__JointStateWithAcceleration__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pronto_msgs__msg__JointStateWithAcceleration__Sequence__are_equal(const pronto_msgs__msg__JointStateWithAcceleration__Sequence * lhs, const pronto_msgs__msg__JointStateWithAcceleration__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pronto_msgs__msg__JointStateWithAcceleration__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pronto_msgs__msg__JointStateWithAcceleration__Sequence__copy(
  const pronto_msgs__msg__JointStateWithAcceleration__Sequence * input,
  pronto_msgs__msg__JointStateWithAcceleration__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pronto_msgs__msg__JointStateWithAcceleration);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pronto_msgs__msg__JointStateWithAcceleration * data =
      (pronto_msgs__msg__JointStateWithAcceleration *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pronto_msgs__msg__JointStateWithAcceleration__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pronto_msgs__msg__JointStateWithAcceleration__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pronto_msgs__msg__JointStateWithAcceleration__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
