// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pronto_msgs:msg/ControllerFootContact.idl
// generated code does not contain a copyright notice
#include "pronto_msgs/msg/detail/controller_foot_contact__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `right_foot_contacts`
// Member `left_foot_contacts`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
pronto_msgs__msg__ControllerFootContact__init(pronto_msgs__msg__ControllerFootContact * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    pronto_msgs__msg__ControllerFootContact__fini(msg);
    return false;
  }
  // num_right_foot_contacts
  // right_foot_contacts
  if (!rosidl_runtime_c__double__Sequence__init(&msg->right_foot_contacts, 0)) {
    pronto_msgs__msg__ControllerFootContact__fini(msg);
    return false;
  }
  // num_left_foot_contacts
  // left_foot_contacts
  if (!rosidl_runtime_c__double__Sequence__init(&msg->left_foot_contacts, 0)) {
    pronto_msgs__msg__ControllerFootContact__fini(msg);
    return false;
  }
  return true;
}

void
pronto_msgs__msg__ControllerFootContact__fini(pronto_msgs__msg__ControllerFootContact * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // num_right_foot_contacts
  // right_foot_contacts
  rosidl_runtime_c__double__Sequence__fini(&msg->right_foot_contacts);
  // num_left_foot_contacts
  // left_foot_contacts
  rosidl_runtime_c__double__Sequence__fini(&msg->left_foot_contacts);
}

pronto_msgs__msg__ControllerFootContact *
pronto_msgs__msg__ControllerFootContact__create()
{
  pronto_msgs__msg__ControllerFootContact * msg = (pronto_msgs__msg__ControllerFootContact *)malloc(sizeof(pronto_msgs__msg__ControllerFootContact));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pronto_msgs__msg__ControllerFootContact));
  bool success = pronto_msgs__msg__ControllerFootContact__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
pronto_msgs__msg__ControllerFootContact__destroy(pronto_msgs__msg__ControllerFootContact * msg)
{
  if (msg) {
    pronto_msgs__msg__ControllerFootContact__fini(msg);
  }
  free(msg);
}


bool
pronto_msgs__msg__ControllerFootContact__Sequence__init(pronto_msgs__msg__ControllerFootContact__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  pronto_msgs__msg__ControllerFootContact * data = NULL;
  if (size) {
    data = (pronto_msgs__msg__ControllerFootContact *)calloc(size, sizeof(pronto_msgs__msg__ControllerFootContact));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pronto_msgs__msg__ControllerFootContact__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pronto_msgs__msg__ControllerFootContact__fini(&data[i - 1]);
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
pronto_msgs__msg__ControllerFootContact__Sequence__fini(pronto_msgs__msg__ControllerFootContact__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      pronto_msgs__msg__ControllerFootContact__fini(&array->data[i]);
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

pronto_msgs__msg__ControllerFootContact__Sequence *
pronto_msgs__msg__ControllerFootContact__Sequence__create(size_t size)
{
  pronto_msgs__msg__ControllerFootContact__Sequence * array = (pronto_msgs__msg__ControllerFootContact__Sequence *)malloc(sizeof(pronto_msgs__msg__ControllerFootContact__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = pronto_msgs__msg__ControllerFootContact__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
pronto_msgs__msg__ControllerFootContact__Sequence__destroy(pronto_msgs__msg__ControllerFootContact__Sequence * array)
{
  if (array) {
    pronto_msgs__msg__ControllerFootContact__Sequence__fini(array);
  }
  free(array);
}
