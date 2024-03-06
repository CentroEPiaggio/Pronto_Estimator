// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pi3hat_moteus_int_msgs:msg/PacketPass.idl
// generated code does not contain a copyright notice
#include "pi3hat_moteus_int_msgs/msg/detail/packet_pass__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `pack_loss`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
pi3hat_moteus_int_msgs__msg__PacketPass__init(pi3hat_moteus_int_msgs__msg__PacketPass * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    pi3hat_moteus_int_msgs__msg__PacketPass__fini(msg);
    return false;
  }
  // valid
  // cycle_dur
  // name
  if (!rosidl_runtime_c__String__Sequence__init(&msg->name, 0)) {
    pi3hat_moteus_int_msgs__msg__PacketPass__fini(msg);
    return false;
  }
  // pack_loss
  if (!rosidl_runtime_c__double__Sequence__init(&msg->pack_loss, 0)) {
    pi3hat_moteus_int_msgs__msg__PacketPass__fini(msg);
    return false;
  }
  return true;
}

void
pi3hat_moteus_int_msgs__msg__PacketPass__fini(pi3hat_moteus_int_msgs__msg__PacketPass * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // valid
  // cycle_dur
  // name
  rosidl_runtime_c__String__Sequence__fini(&msg->name);
  // pack_loss
  rosidl_runtime_c__double__Sequence__fini(&msg->pack_loss);
}

pi3hat_moteus_int_msgs__msg__PacketPass *
pi3hat_moteus_int_msgs__msg__PacketPass__create()
{
  pi3hat_moteus_int_msgs__msg__PacketPass * msg = (pi3hat_moteus_int_msgs__msg__PacketPass *)malloc(sizeof(pi3hat_moteus_int_msgs__msg__PacketPass));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pi3hat_moteus_int_msgs__msg__PacketPass));
  bool success = pi3hat_moteus_int_msgs__msg__PacketPass__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
pi3hat_moteus_int_msgs__msg__PacketPass__destroy(pi3hat_moteus_int_msgs__msg__PacketPass * msg)
{
  if (msg) {
    pi3hat_moteus_int_msgs__msg__PacketPass__fini(msg);
  }
  free(msg);
}


bool
pi3hat_moteus_int_msgs__msg__PacketPass__Sequence__init(pi3hat_moteus_int_msgs__msg__PacketPass__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  pi3hat_moteus_int_msgs__msg__PacketPass * data = NULL;
  if (size) {
    data = (pi3hat_moteus_int_msgs__msg__PacketPass *)calloc(size, sizeof(pi3hat_moteus_int_msgs__msg__PacketPass));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pi3hat_moteus_int_msgs__msg__PacketPass__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pi3hat_moteus_int_msgs__msg__PacketPass__fini(&data[i - 1]);
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
pi3hat_moteus_int_msgs__msg__PacketPass__Sequence__fini(pi3hat_moteus_int_msgs__msg__PacketPass__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      pi3hat_moteus_int_msgs__msg__PacketPass__fini(&array->data[i]);
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

pi3hat_moteus_int_msgs__msg__PacketPass__Sequence *
pi3hat_moteus_int_msgs__msg__PacketPass__Sequence__create(size_t size)
{
  pi3hat_moteus_int_msgs__msg__PacketPass__Sequence * array = (pi3hat_moteus_int_msgs__msg__PacketPass__Sequence *)malloc(sizeof(pi3hat_moteus_int_msgs__msg__PacketPass__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = pi3hat_moteus_int_msgs__msg__PacketPass__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
pi3hat_moteus_int_msgs__msg__PacketPass__Sequence__destroy(pi3hat_moteus_int_msgs__msg__PacketPass__Sequence * array)
{
  if (array) {
    pi3hat_moteus_int_msgs__msg__PacketPass__Sequence__fini(array);
  }
  free(array);
}
