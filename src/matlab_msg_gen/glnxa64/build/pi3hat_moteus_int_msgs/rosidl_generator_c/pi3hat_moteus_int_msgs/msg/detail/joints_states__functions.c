// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pi3hat_moteus_int_msgs:msg/JointsStates.idl
// generated code does not contain a copyright notice
#include "pi3hat_moteus_int_msgs/msg/detail/joints_states__functions.h"

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
// Member `current`
// Member `temperature`
// Member `sec_enc_pos`
// Member `sec_enc_vel`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
pi3hat_moteus_int_msgs__msg__JointsStates__init(pi3hat_moteus_int_msgs__msg__JointsStates * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    pi3hat_moteus_int_msgs__msg__JointsStates__fini(msg);
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__Sequence__init(&msg->name, 0)) {
    pi3hat_moteus_int_msgs__msg__JointsStates__fini(msg);
    return false;
  }
  // position
  if (!rosidl_runtime_c__double__Sequence__init(&msg->position, 0)) {
    pi3hat_moteus_int_msgs__msg__JointsStates__fini(msg);
    return false;
  }
  // velocity
  if (!rosidl_runtime_c__double__Sequence__init(&msg->velocity, 0)) {
    pi3hat_moteus_int_msgs__msg__JointsStates__fini(msg);
    return false;
  }
  // effort
  if (!rosidl_runtime_c__double__Sequence__init(&msg->effort, 0)) {
    pi3hat_moteus_int_msgs__msg__JointsStates__fini(msg);
    return false;
  }
  // current
  if (!rosidl_runtime_c__double__Sequence__init(&msg->current, 0)) {
    pi3hat_moteus_int_msgs__msg__JointsStates__fini(msg);
    return false;
  }
  // temperature
  if (!rosidl_runtime_c__double__Sequence__init(&msg->temperature, 0)) {
    pi3hat_moteus_int_msgs__msg__JointsStates__fini(msg);
    return false;
  }
  // sec_enc_pos
  if (!rosidl_runtime_c__double__Sequence__init(&msg->sec_enc_pos, 0)) {
    pi3hat_moteus_int_msgs__msg__JointsStates__fini(msg);
    return false;
  }
  // sec_enc_vel
  if (!rosidl_runtime_c__double__Sequence__init(&msg->sec_enc_vel, 0)) {
    pi3hat_moteus_int_msgs__msg__JointsStates__fini(msg);
    return false;
  }
  return true;
}

void
pi3hat_moteus_int_msgs__msg__JointsStates__fini(pi3hat_moteus_int_msgs__msg__JointsStates * msg)
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
  // current
  rosidl_runtime_c__double__Sequence__fini(&msg->current);
  // temperature
  rosidl_runtime_c__double__Sequence__fini(&msg->temperature);
  // sec_enc_pos
  rosidl_runtime_c__double__Sequence__fini(&msg->sec_enc_pos);
  // sec_enc_vel
  rosidl_runtime_c__double__Sequence__fini(&msg->sec_enc_vel);
}

pi3hat_moteus_int_msgs__msg__JointsStates *
pi3hat_moteus_int_msgs__msg__JointsStates__create()
{
  pi3hat_moteus_int_msgs__msg__JointsStates * msg = (pi3hat_moteus_int_msgs__msg__JointsStates *)malloc(sizeof(pi3hat_moteus_int_msgs__msg__JointsStates));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pi3hat_moteus_int_msgs__msg__JointsStates));
  bool success = pi3hat_moteus_int_msgs__msg__JointsStates__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
pi3hat_moteus_int_msgs__msg__JointsStates__destroy(pi3hat_moteus_int_msgs__msg__JointsStates * msg)
{
  if (msg) {
    pi3hat_moteus_int_msgs__msg__JointsStates__fini(msg);
  }
  free(msg);
}


bool
pi3hat_moteus_int_msgs__msg__JointsStates__Sequence__init(pi3hat_moteus_int_msgs__msg__JointsStates__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  pi3hat_moteus_int_msgs__msg__JointsStates * data = NULL;
  if (size) {
    data = (pi3hat_moteus_int_msgs__msg__JointsStates *)calloc(size, sizeof(pi3hat_moteus_int_msgs__msg__JointsStates));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pi3hat_moteus_int_msgs__msg__JointsStates__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pi3hat_moteus_int_msgs__msg__JointsStates__fini(&data[i - 1]);
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
pi3hat_moteus_int_msgs__msg__JointsStates__Sequence__fini(pi3hat_moteus_int_msgs__msg__JointsStates__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      pi3hat_moteus_int_msgs__msg__JointsStates__fini(&array->data[i]);
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

pi3hat_moteus_int_msgs__msg__JointsStates__Sequence *
pi3hat_moteus_int_msgs__msg__JointsStates__Sequence__create(size_t size)
{
  pi3hat_moteus_int_msgs__msg__JointsStates__Sequence * array = (pi3hat_moteus_int_msgs__msg__JointsStates__Sequence *)malloc(sizeof(pi3hat_moteus_int_msgs__msg__JointsStates__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = pi3hat_moteus_int_msgs__msg__JointsStates__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
pi3hat_moteus_int_msgs__msg__JointsStates__Sequence__destroy(pi3hat_moteus_int_msgs__msg__JointsStates__Sequence * array)
{
  if (array) {
    pi3hat_moteus_int_msgs__msg__JointsStates__Sequence__fini(array);
  }
  free(array);
}
