// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pronto_msgs:msg/VelocityWithSigmaBounds.idl
// generated code does not contain a copyright notice
#include "pronto_msgs/msg/detail/velocity_with_sigma_bounds__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `velocity_plus_one_sigma`
// Member `velocity_minus_one_sigma`
// Member `plus_one_sigma`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
pronto_msgs__msg__VelocityWithSigmaBounds__init(pronto_msgs__msg__VelocityWithSigmaBounds * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    pronto_msgs__msg__VelocityWithSigmaBounds__fini(msg);
    return false;
  }
  // velocity_plus_one_sigma
  if (!geometry_msgs__msg__Vector3__init(&msg->velocity_plus_one_sigma)) {
    pronto_msgs__msg__VelocityWithSigmaBounds__fini(msg);
    return false;
  }
  // velocity_minus_one_sigma
  if (!geometry_msgs__msg__Vector3__init(&msg->velocity_minus_one_sigma)) {
    pronto_msgs__msg__VelocityWithSigmaBounds__fini(msg);
    return false;
  }
  // plus_one_sigma
  if (!geometry_msgs__msg__Vector3__init(&msg->plus_one_sigma)) {
    pronto_msgs__msg__VelocityWithSigmaBounds__fini(msg);
    return false;
  }
  return true;
}

void
pronto_msgs__msg__VelocityWithSigmaBounds__fini(pronto_msgs__msg__VelocityWithSigmaBounds * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // velocity_plus_one_sigma
  geometry_msgs__msg__Vector3__fini(&msg->velocity_plus_one_sigma);
  // velocity_minus_one_sigma
  geometry_msgs__msg__Vector3__fini(&msg->velocity_minus_one_sigma);
  // plus_one_sigma
  geometry_msgs__msg__Vector3__fini(&msg->plus_one_sigma);
}

pronto_msgs__msg__VelocityWithSigmaBounds *
pronto_msgs__msg__VelocityWithSigmaBounds__create()
{
  pronto_msgs__msg__VelocityWithSigmaBounds * msg = (pronto_msgs__msg__VelocityWithSigmaBounds *)malloc(sizeof(pronto_msgs__msg__VelocityWithSigmaBounds));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pronto_msgs__msg__VelocityWithSigmaBounds));
  bool success = pronto_msgs__msg__VelocityWithSigmaBounds__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
pronto_msgs__msg__VelocityWithSigmaBounds__destroy(pronto_msgs__msg__VelocityWithSigmaBounds * msg)
{
  if (msg) {
    pronto_msgs__msg__VelocityWithSigmaBounds__fini(msg);
  }
  free(msg);
}


bool
pronto_msgs__msg__VelocityWithSigmaBounds__Sequence__init(pronto_msgs__msg__VelocityWithSigmaBounds__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  pronto_msgs__msg__VelocityWithSigmaBounds * data = NULL;
  if (size) {
    data = (pronto_msgs__msg__VelocityWithSigmaBounds *)calloc(size, sizeof(pronto_msgs__msg__VelocityWithSigmaBounds));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pronto_msgs__msg__VelocityWithSigmaBounds__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pronto_msgs__msg__VelocityWithSigmaBounds__fini(&data[i - 1]);
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
pronto_msgs__msg__VelocityWithSigmaBounds__Sequence__fini(pronto_msgs__msg__VelocityWithSigmaBounds__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      pronto_msgs__msg__VelocityWithSigmaBounds__fini(&array->data[i]);
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

pronto_msgs__msg__VelocityWithSigmaBounds__Sequence *
pronto_msgs__msg__VelocityWithSigmaBounds__Sequence__create(size_t size)
{
  pronto_msgs__msg__VelocityWithSigmaBounds__Sequence * array = (pronto_msgs__msg__VelocityWithSigmaBounds__Sequence *)malloc(sizeof(pronto_msgs__msg__VelocityWithSigmaBounds__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = pronto_msgs__msg__VelocityWithSigmaBounds__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
pronto_msgs__msg__VelocityWithSigmaBounds__Sequence__destroy(pronto_msgs__msg__VelocityWithSigmaBounds__Sequence * array)
{
  if (array) {
    pronto_msgs__msg__VelocityWithSigmaBounds__Sequence__fini(array);
  }
  free(array);
}
