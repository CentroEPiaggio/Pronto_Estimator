// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pronto_msgs:msg/JointStateWithAcceleration.idl
// generated code does not contain a copyright notice
#include "pronto_msgs/msg/detail/joint_state_with_acceleration__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


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

pronto_msgs__msg__JointStateWithAcceleration *
pronto_msgs__msg__JointStateWithAcceleration__create()
{
  pronto_msgs__msg__JointStateWithAcceleration * msg = (pronto_msgs__msg__JointStateWithAcceleration *)malloc(sizeof(pronto_msgs__msg__JointStateWithAcceleration));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pronto_msgs__msg__JointStateWithAcceleration));
  bool success = pronto_msgs__msg__JointStateWithAcceleration__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
pronto_msgs__msg__JointStateWithAcceleration__destroy(pronto_msgs__msg__JointStateWithAcceleration * msg)
{
  if (msg) {
    pronto_msgs__msg__JointStateWithAcceleration__fini(msg);
  }
  free(msg);
}


bool
pronto_msgs__msg__JointStateWithAcceleration__Sequence__init(pronto_msgs__msg__JointStateWithAcceleration__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  pronto_msgs__msg__JointStateWithAcceleration * data = NULL;
  if (size) {
    data = (pronto_msgs__msg__JointStateWithAcceleration *)calloc(size, sizeof(pronto_msgs__msg__JointStateWithAcceleration));
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
pronto_msgs__msg__JointStateWithAcceleration__Sequence__fini(pronto_msgs__msg__JointStateWithAcceleration__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      pronto_msgs__msg__JointStateWithAcceleration__fini(&array->data[i]);
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

pronto_msgs__msg__JointStateWithAcceleration__Sequence *
pronto_msgs__msg__JointStateWithAcceleration__Sequence__create(size_t size)
{
  pronto_msgs__msg__JointStateWithAcceleration__Sequence * array = (pronto_msgs__msg__JointStateWithAcceleration__Sequence *)malloc(sizeof(pronto_msgs__msg__JointStateWithAcceleration__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = pronto_msgs__msg__JointStateWithAcceleration__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
pronto_msgs__msg__JointStateWithAcceleration__Sequence__destroy(pronto_msgs__msg__JointStateWithAcceleration__Sequence * array)
{
  if (array) {
    pronto_msgs__msg__JointStateWithAcceleration__Sequence__fini(array);
  }
  free(array);
}
