// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pi3hat_moteus_int_msgs:msg/JointsCommand.idl
// generated code does not contain a copyright notice
#include "pi3hat_moteus_int_msgs/msg/detail/joints_command__functions.h"

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
// Member `effort`
// Member `kp_scale`
// Member `kd_scale`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
pi3hat_moteus_int_msgs__msg__JointsCommand__init(pi3hat_moteus_int_msgs__msg__JointsCommand * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    pi3hat_moteus_int_msgs__msg__JointsCommand__fini(msg);
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__Sequence__init(&msg->name, 0)) {
    pi3hat_moteus_int_msgs__msg__JointsCommand__fini(msg);
    return false;
  }
  // position
  if (!rosidl_runtime_c__double__Sequence__init(&msg->position, 0)) {
    pi3hat_moteus_int_msgs__msg__JointsCommand__fini(msg);
    return false;
  }
  // velocity
  if (!rosidl_runtime_c__double__Sequence__init(&msg->velocity, 0)) {
    pi3hat_moteus_int_msgs__msg__JointsCommand__fini(msg);
    return false;
  }
  // effort
  if (!rosidl_runtime_c__double__Sequence__init(&msg->effort, 0)) {
    pi3hat_moteus_int_msgs__msg__JointsCommand__fini(msg);
    return false;
  }
  // kp_scale
  if (!rosidl_runtime_c__double__Sequence__init(&msg->kp_scale, 0)) {
    pi3hat_moteus_int_msgs__msg__JointsCommand__fini(msg);
    return false;
  }
  // kd_scale
  if (!rosidl_runtime_c__double__Sequence__init(&msg->kd_scale, 0)) {
    pi3hat_moteus_int_msgs__msg__JointsCommand__fini(msg);
    return false;
  }
  return true;
}

void
pi3hat_moteus_int_msgs__msg__JointsCommand__fini(pi3hat_moteus_int_msgs__msg__JointsCommand * msg)
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
  // effort
  rosidl_runtime_c__double__Sequence__fini(&msg->effort);
  // kp_scale
  rosidl_runtime_c__double__Sequence__fini(&msg->kp_scale);
  // kd_scale
  rosidl_runtime_c__double__Sequence__fini(&msg->kd_scale);
}

pi3hat_moteus_int_msgs__msg__JointsCommand *
pi3hat_moteus_int_msgs__msg__JointsCommand__create()
{
  pi3hat_moteus_int_msgs__msg__JointsCommand * msg = (pi3hat_moteus_int_msgs__msg__JointsCommand *)malloc(sizeof(pi3hat_moteus_int_msgs__msg__JointsCommand));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pi3hat_moteus_int_msgs__msg__JointsCommand));
  bool success = pi3hat_moteus_int_msgs__msg__JointsCommand__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
pi3hat_moteus_int_msgs__msg__JointsCommand__destroy(pi3hat_moteus_int_msgs__msg__JointsCommand * msg)
{
  if (msg) {
    pi3hat_moteus_int_msgs__msg__JointsCommand__fini(msg);
  }
  free(msg);
}


bool
pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence__init(pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  pi3hat_moteus_int_msgs__msg__JointsCommand * data = NULL;
  if (size) {
    data = (pi3hat_moteus_int_msgs__msg__JointsCommand *)calloc(size, sizeof(pi3hat_moteus_int_msgs__msg__JointsCommand));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pi3hat_moteus_int_msgs__msg__JointsCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pi3hat_moteus_int_msgs__msg__JointsCommand__fini(&data[i - 1]);
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
pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence__fini(pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      pi3hat_moteus_int_msgs__msg__JointsCommand__fini(&array->data[i]);
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

pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence *
pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence__create(size_t size)
{
  pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence * array = (pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence *)malloc(sizeof(pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence__destroy(pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence * array)
{
  if (array) {
    pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence__fini(array);
  }
  free(array);
}
