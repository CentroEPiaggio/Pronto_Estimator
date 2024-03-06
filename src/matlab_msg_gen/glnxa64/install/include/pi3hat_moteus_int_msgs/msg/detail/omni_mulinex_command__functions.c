// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pi3hat_moteus_int_msgs:msg/OmniMulinexCommand.idl
// generated code does not contain a copyright notice
#include "pi3hat_moteus_int_msgs/msg/detail/omni_mulinex_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
pi3hat_moteus_int_msgs__msg__OmniMulinexCommand__init(pi3hat_moteus_int_msgs__msg__OmniMulinexCommand * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    pi3hat_moteus_int_msgs__msg__OmniMulinexCommand__fini(msg);
    return false;
  }
  // v_x
  // v_y
  // omega
  // height_rate
  return true;
}

void
pi3hat_moteus_int_msgs__msg__OmniMulinexCommand__fini(pi3hat_moteus_int_msgs__msg__OmniMulinexCommand * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // v_x
  // v_y
  // omega
  // height_rate
}

pi3hat_moteus_int_msgs__msg__OmniMulinexCommand *
pi3hat_moteus_int_msgs__msg__OmniMulinexCommand__create()
{
  pi3hat_moteus_int_msgs__msg__OmniMulinexCommand * msg = (pi3hat_moteus_int_msgs__msg__OmniMulinexCommand *)malloc(sizeof(pi3hat_moteus_int_msgs__msg__OmniMulinexCommand));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pi3hat_moteus_int_msgs__msg__OmniMulinexCommand));
  bool success = pi3hat_moteus_int_msgs__msg__OmniMulinexCommand__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
pi3hat_moteus_int_msgs__msg__OmniMulinexCommand__destroy(pi3hat_moteus_int_msgs__msg__OmniMulinexCommand * msg)
{
  if (msg) {
    pi3hat_moteus_int_msgs__msg__OmniMulinexCommand__fini(msg);
  }
  free(msg);
}


bool
pi3hat_moteus_int_msgs__msg__OmniMulinexCommand__Sequence__init(pi3hat_moteus_int_msgs__msg__OmniMulinexCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  pi3hat_moteus_int_msgs__msg__OmniMulinexCommand * data = NULL;
  if (size) {
    data = (pi3hat_moteus_int_msgs__msg__OmniMulinexCommand *)calloc(size, sizeof(pi3hat_moteus_int_msgs__msg__OmniMulinexCommand));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pi3hat_moteus_int_msgs__msg__OmniMulinexCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pi3hat_moteus_int_msgs__msg__OmniMulinexCommand__fini(&data[i - 1]);
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
pi3hat_moteus_int_msgs__msg__OmniMulinexCommand__Sequence__fini(pi3hat_moteus_int_msgs__msg__OmniMulinexCommand__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      pi3hat_moteus_int_msgs__msg__OmniMulinexCommand__fini(&array->data[i]);
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

pi3hat_moteus_int_msgs__msg__OmniMulinexCommand__Sequence *
pi3hat_moteus_int_msgs__msg__OmniMulinexCommand__Sequence__create(size_t size)
{
  pi3hat_moteus_int_msgs__msg__OmniMulinexCommand__Sequence * array = (pi3hat_moteus_int_msgs__msg__OmniMulinexCommand__Sequence *)malloc(sizeof(pi3hat_moteus_int_msgs__msg__OmniMulinexCommand__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = pi3hat_moteus_int_msgs__msg__OmniMulinexCommand__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
pi3hat_moteus_int_msgs__msg__OmniMulinexCommand__Sequence__destroy(pi3hat_moteus_int_msgs__msg__OmniMulinexCommand__Sequence * array)
{
  if (array) {
    pi3hat_moteus_int_msgs__msg__OmniMulinexCommand__Sequence__fini(array);
  }
  free(array);
}
