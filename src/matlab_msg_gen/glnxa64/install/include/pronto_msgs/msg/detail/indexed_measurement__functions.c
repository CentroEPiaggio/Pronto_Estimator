// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pronto_msgs:msg/IndexedMeasurement.idl
// generated code does not contain a copyright notice
#include "pronto_msgs/msg/detail/indexed_measurement__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `z_effective`
// Member `z_indices`
// Member `r_effective`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
pronto_msgs__msg__IndexedMeasurement__init(pronto_msgs__msg__IndexedMeasurement * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    pronto_msgs__msg__IndexedMeasurement__fini(msg);
    return false;
  }
  // utime
  // state_utime
  // z_effective
  if (!rosidl_runtime_c__double__Sequence__init(&msg->z_effective, 0)) {
    pronto_msgs__msg__IndexedMeasurement__fini(msg);
    return false;
  }
  // z_indices
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->z_indices, 0)) {
    pronto_msgs__msg__IndexedMeasurement__fini(msg);
    return false;
  }
  // r_effective
  if (!rosidl_runtime_c__double__Sequence__init(&msg->r_effective, 0)) {
    pronto_msgs__msg__IndexedMeasurement__fini(msg);
    return false;
  }
  return true;
}

void
pronto_msgs__msg__IndexedMeasurement__fini(pronto_msgs__msg__IndexedMeasurement * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // utime
  // state_utime
  // z_effective
  rosidl_runtime_c__double__Sequence__fini(&msg->z_effective);
  // z_indices
  rosidl_runtime_c__int32__Sequence__fini(&msg->z_indices);
  // r_effective
  rosidl_runtime_c__double__Sequence__fini(&msg->r_effective);
}

pronto_msgs__msg__IndexedMeasurement *
pronto_msgs__msg__IndexedMeasurement__create()
{
  pronto_msgs__msg__IndexedMeasurement * msg = (pronto_msgs__msg__IndexedMeasurement *)malloc(sizeof(pronto_msgs__msg__IndexedMeasurement));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pronto_msgs__msg__IndexedMeasurement));
  bool success = pronto_msgs__msg__IndexedMeasurement__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
pronto_msgs__msg__IndexedMeasurement__destroy(pronto_msgs__msg__IndexedMeasurement * msg)
{
  if (msg) {
    pronto_msgs__msg__IndexedMeasurement__fini(msg);
  }
  free(msg);
}


bool
pronto_msgs__msg__IndexedMeasurement__Sequence__init(pronto_msgs__msg__IndexedMeasurement__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  pronto_msgs__msg__IndexedMeasurement * data = NULL;
  if (size) {
    data = (pronto_msgs__msg__IndexedMeasurement *)calloc(size, sizeof(pronto_msgs__msg__IndexedMeasurement));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pronto_msgs__msg__IndexedMeasurement__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pronto_msgs__msg__IndexedMeasurement__fini(&data[i - 1]);
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
pronto_msgs__msg__IndexedMeasurement__Sequence__fini(pronto_msgs__msg__IndexedMeasurement__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      pronto_msgs__msg__IndexedMeasurement__fini(&array->data[i]);
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

pronto_msgs__msg__IndexedMeasurement__Sequence *
pronto_msgs__msg__IndexedMeasurement__Sequence__create(size_t size)
{
  pronto_msgs__msg__IndexedMeasurement__Sequence * array = (pronto_msgs__msg__IndexedMeasurement__Sequence *)malloc(sizeof(pronto_msgs__msg__IndexedMeasurement__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = pronto_msgs__msg__IndexedMeasurement__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
pronto_msgs__msg__IndexedMeasurement__Sequence__destroy(pronto_msgs__msg__IndexedMeasurement__Sequence * array)
{
  if (array) {
    pronto_msgs__msg__IndexedMeasurement__Sequence__fini(array);
  }
  free(array);
}
