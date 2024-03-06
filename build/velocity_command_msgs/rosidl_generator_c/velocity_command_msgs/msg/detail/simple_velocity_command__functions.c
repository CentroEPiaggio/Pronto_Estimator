// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from velocity_command_msgs:msg/SimpleVelocityCommand.idl
// generated code does not contain a copyright notice
#include "velocity_command_msgs/msg/detail/simple_velocity_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
velocity_command_msgs__msg__SimpleVelocityCommand__init(velocity_command_msgs__msg__SimpleVelocityCommand * msg)
{
  if (!msg) {
    return false;
  }
  // velocity_forward
  // velocity_lateral
  // yaw_rate
  return true;
}

void
velocity_command_msgs__msg__SimpleVelocityCommand__fini(velocity_command_msgs__msg__SimpleVelocityCommand * msg)
{
  if (!msg) {
    return;
  }
  // velocity_forward
  // velocity_lateral
  // yaw_rate
}

bool
velocity_command_msgs__msg__SimpleVelocityCommand__are_equal(const velocity_command_msgs__msg__SimpleVelocityCommand * lhs, const velocity_command_msgs__msg__SimpleVelocityCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // velocity_forward
  if (lhs->velocity_forward != rhs->velocity_forward) {
    return false;
  }
  // velocity_lateral
  if (lhs->velocity_lateral != rhs->velocity_lateral) {
    return false;
  }
  // yaw_rate
  if (lhs->yaw_rate != rhs->yaw_rate) {
    return false;
  }
  return true;
}

bool
velocity_command_msgs__msg__SimpleVelocityCommand__copy(
  const velocity_command_msgs__msg__SimpleVelocityCommand * input,
  velocity_command_msgs__msg__SimpleVelocityCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // velocity_forward
  output->velocity_forward = input->velocity_forward;
  // velocity_lateral
  output->velocity_lateral = input->velocity_lateral;
  // yaw_rate
  output->yaw_rate = input->yaw_rate;
  return true;
}

velocity_command_msgs__msg__SimpleVelocityCommand *
velocity_command_msgs__msg__SimpleVelocityCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  velocity_command_msgs__msg__SimpleVelocityCommand * msg = (velocity_command_msgs__msg__SimpleVelocityCommand *)allocator.allocate(sizeof(velocity_command_msgs__msg__SimpleVelocityCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(velocity_command_msgs__msg__SimpleVelocityCommand));
  bool success = velocity_command_msgs__msg__SimpleVelocityCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
velocity_command_msgs__msg__SimpleVelocityCommand__destroy(velocity_command_msgs__msg__SimpleVelocityCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    velocity_command_msgs__msg__SimpleVelocityCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
velocity_command_msgs__msg__SimpleVelocityCommand__Sequence__init(velocity_command_msgs__msg__SimpleVelocityCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  velocity_command_msgs__msg__SimpleVelocityCommand * data = NULL;

  if (size) {
    data = (velocity_command_msgs__msg__SimpleVelocityCommand *)allocator.zero_allocate(size, sizeof(velocity_command_msgs__msg__SimpleVelocityCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = velocity_command_msgs__msg__SimpleVelocityCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        velocity_command_msgs__msg__SimpleVelocityCommand__fini(&data[i - 1]);
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
velocity_command_msgs__msg__SimpleVelocityCommand__Sequence__fini(velocity_command_msgs__msg__SimpleVelocityCommand__Sequence * array)
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
      velocity_command_msgs__msg__SimpleVelocityCommand__fini(&array->data[i]);
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

velocity_command_msgs__msg__SimpleVelocityCommand__Sequence *
velocity_command_msgs__msg__SimpleVelocityCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  velocity_command_msgs__msg__SimpleVelocityCommand__Sequence * array = (velocity_command_msgs__msg__SimpleVelocityCommand__Sequence *)allocator.allocate(sizeof(velocity_command_msgs__msg__SimpleVelocityCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = velocity_command_msgs__msg__SimpleVelocityCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
velocity_command_msgs__msg__SimpleVelocityCommand__Sequence__destroy(velocity_command_msgs__msg__SimpleVelocityCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    velocity_command_msgs__msg__SimpleVelocityCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
velocity_command_msgs__msg__SimpleVelocityCommand__Sequence__are_equal(const velocity_command_msgs__msg__SimpleVelocityCommand__Sequence * lhs, const velocity_command_msgs__msg__SimpleVelocityCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!velocity_command_msgs__msg__SimpleVelocityCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
velocity_command_msgs__msg__SimpleVelocityCommand__Sequence__copy(
  const velocity_command_msgs__msg__SimpleVelocityCommand__Sequence * input,
  velocity_command_msgs__msg__SimpleVelocityCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(velocity_command_msgs__msg__SimpleVelocityCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    velocity_command_msgs__msg__SimpleVelocityCommand * data =
      (velocity_command_msgs__msg__SimpleVelocityCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!velocity_command_msgs__msg__SimpleVelocityCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          velocity_command_msgs__msg__SimpleVelocityCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!velocity_command_msgs__msg__SimpleVelocityCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
