// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pronto_msgs:msg/FilterState.idl
// generated code does not contain a copyright notice
#include "pronto_msgs/msg/detail/filter_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `quat`
#include "geometry_msgs/msg/detail/quaternion__functions.h"
// Member `state`
// Member `cov`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
pronto_msgs__msg__FilterState__init(pronto_msgs__msg__FilterState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    pronto_msgs__msg__FilterState__fini(msg);
    return false;
  }
  // quat
  if (!geometry_msgs__msg__Quaternion__init(&msg->quat)) {
    pronto_msgs__msg__FilterState__fini(msg);
    return false;
  }
  // state
  if (!rosidl_runtime_c__double__Sequence__init(&msg->state, 0)) {
    pronto_msgs__msg__FilterState__fini(msg);
    return false;
  }
  // cov
  if (!rosidl_runtime_c__double__Sequence__init(&msg->cov, 0)) {
    pronto_msgs__msg__FilterState__fini(msg);
    return false;
  }
  return true;
}

void
pronto_msgs__msg__FilterState__fini(pronto_msgs__msg__FilterState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // quat
  geometry_msgs__msg__Quaternion__fini(&msg->quat);
  // state
  rosidl_runtime_c__double__Sequence__fini(&msg->state);
  // cov
  rosidl_runtime_c__double__Sequence__fini(&msg->cov);
}

pronto_msgs__msg__FilterState *
pronto_msgs__msg__FilterState__create()
{
  pronto_msgs__msg__FilterState * msg = (pronto_msgs__msg__FilterState *)malloc(sizeof(pronto_msgs__msg__FilterState));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pronto_msgs__msg__FilterState));
  bool success = pronto_msgs__msg__FilterState__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
pronto_msgs__msg__FilterState__destroy(pronto_msgs__msg__FilterState * msg)
{
  if (msg) {
    pronto_msgs__msg__FilterState__fini(msg);
  }
  free(msg);
}


bool
pronto_msgs__msg__FilterState__Sequence__init(pronto_msgs__msg__FilterState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  pronto_msgs__msg__FilterState * data = NULL;
  if (size) {
    data = (pronto_msgs__msg__FilterState *)calloc(size, sizeof(pronto_msgs__msg__FilterState));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pronto_msgs__msg__FilterState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pronto_msgs__msg__FilterState__fini(&data[i - 1]);
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
pronto_msgs__msg__FilterState__Sequence__fini(pronto_msgs__msg__FilterState__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      pronto_msgs__msg__FilterState__fini(&array->data[i]);
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

pronto_msgs__msg__FilterState__Sequence *
pronto_msgs__msg__FilterState__Sequence__create(size_t size)
{
  pronto_msgs__msg__FilterState__Sequence * array = (pronto_msgs__msg__FilterState__Sequence *)malloc(sizeof(pronto_msgs__msg__FilterState__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = pronto_msgs__msg__FilterState__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
pronto_msgs__msg__FilterState__Sequence__destroy(pronto_msgs__msg__FilterState__Sequence * array)
{
  if (array) {
    pronto_msgs__msg__FilterState__Sequence__fini(array);
  }
  free(array);
}
