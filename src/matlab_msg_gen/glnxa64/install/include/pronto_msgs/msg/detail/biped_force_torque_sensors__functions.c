// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pronto_msgs:msg/BipedForceTorqueSensors.idl
// generated code does not contain a copyright notice
#include "pronto_msgs/msg/detail/biped_force_torque_sensors__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `l_foot`
// Member `r_foot`
// Member `l_hand`
// Member `r_hand`
#include "geometry_msgs/msg/detail/wrench__functions.h"

bool
pronto_msgs__msg__BipedForceTorqueSensors__init(pronto_msgs__msg__BipedForceTorqueSensors * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    pronto_msgs__msg__BipedForceTorqueSensors__fini(msg);
    return false;
  }
  // l_foot
  if (!geometry_msgs__msg__Wrench__init(&msg->l_foot)) {
    pronto_msgs__msg__BipedForceTorqueSensors__fini(msg);
    return false;
  }
  // r_foot
  if (!geometry_msgs__msg__Wrench__init(&msg->r_foot)) {
    pronto_msgs__msg__BipedForceTorqueSensors__fini(msg);
    return false;
  }
  // l_hand
  if (!geometry_msgs__msg__Wrench__init(&msg->l_hand)) {
    pronto_msgs__msg__BipedForceTorqueSensors__fini(msg);
    return false;
  }
  // r_hand
  if (!geometry_msgs__msg__Wrench__init(&msg->r_hand)) {
    pronto_msgs__msg__BipedForceTorqueSensors__fini(msg);
    return false;
  }
  return true;
}

void
pronto_msgs__msg__BipedForceTorqueSensors__fini(pronto_msgs__msg__BipedForceTorqueSensors * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // l_foot
  geometry_msgs__msg__Wrench__fini(&msg->l_foot);
  // r_foot
  geometry_msgs__msg__Wrench__fini(&msg->r_foot);
  // l_hand
  geometry_msgs__msg__Wrench__fini(&msg->l_hand);
  // r_hand
  geometry_msgs__msg__Wrench__fini(&msg->r_hand);
}

pronto_msgs__msg__BipedForceTorqueSensors *
pronto_msgs__msg__BipedForceTorqueSensors__create()
{
  pronto_msgs__msg__BipedForceTorqueSensors * msg = (pronto_msgs__msg__BipedForceTorqueSensors *)malloc(sizeof(pronto_msgs__msg__BipedForceTorqueSensors));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pronto_msgs__msg__BipedForceTorqueSensors));
  bool success = pronto_msgs__msg__BipedForceTorqueSensors__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
pronto_msgs__msg__BipedForceTorqueSensors__destroy(pronto_msgs__msg__BipedForceTorqueSensors * msg)
{
  if (msg) {
    pronto_msgs__msg__BipedForceTorqueSensors__fini(msg);
  }
  free(msg);
}


bool
pronto_msgs__msg__BipedForceTorqueSensors__Sequence__init(pronto_msgs__msg__BipedForceTorqueSensors__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  pronto_msgs__msg__BipedForceTorqueSensors * data = NULL;
  if (size) {
    data = (pronto_msgs__msg__BipedForceTorqueSensors *)calloc(size, sizeof(pronto_msgs__msg__BipedForceTorqueSensors));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pronto_msgs__msg__BipedForceTorqueSensors__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pronto_msgs__msg__BipedForceTorqueSensors__fini(&data[i - 1]);
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
pronto_msgs__msg__BipedForceTorqueSensors__Sequence__fini(pronto_msgs__msg__BipedForceTorqueSensors__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      pronto_msgs__msg__BipedForceTorqueSensors__fini(&array->data[i]);
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

pronto_msgs__msg__BipedForceTorqueSensors__Sequence *
pronto_msgs__msg__BipedForceTorqueSensors__Sequence__create(size_t size)
{
  pronto_msgs__msg__BipedForceTorqueSensors__Sequence * array = (pronto_msgs__msg__BipedForceTorqueSensors__Sequence *)malloc(sizeof(pronto_msgs__msg__BipedForceTorqueSensors__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = pronto_msgs__msg__BipedForceTorqueSensors__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
pronto_msgs__msg__BipedForceTorqueSensors__Sequence__destroy(pronto_msgs__msg__BipedForceTorqueSensors__Sequence * array)
{
  if (array) {
    pronto_msgs__msg__BipedForceTorqueSensors__Sequence__fini(array);
  }
  free(array);
}
