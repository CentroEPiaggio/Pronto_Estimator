// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from generalized_pose_msgs:msg/GeneralizedPose.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "generalized_pose_msgs/msg/detail/generalized_pose__rosidl_typesupport_introspection_c.h"
#include "generalized_pose_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "generalized_pose_msgs/msg/detail/generalized_pose__functions.h"
#include "generalized_pose_msgs/msg/detail/generalized_pose__struct.h"


// Include directives for member types
// Member `base_acc`
// Member `base_vel`
// Member `base_pos`
// Member `base_angvel`
#include "geometry_msgs/msg/vector3.h"
// Member `base_acc`
// Member `base_vel`
// Member `base_pos`
// Member `base_angvel`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"
// Member `base_quat`
#include "geometry_msgs/msg/quaternion.h"
// Member `base_quat`
#include "geometry_msgs/msg/detail/quaternion__rosidl_typesupport_introspection_c.h"
// Member `feet_acc`
// Member `feet_vel`
// Member `feet_pos`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `contact_feet`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__GeneralizedPose_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  generalized_pose_msgs__msg__GeneralizedPose__init(message_memory);
}

void generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__GeneralizedPose_fini_function(void * message_memory)
{
  generalized_pose_msgs__msg__GeneralizedPose__fini(message_memory);
}

size_t generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__size_function__GeneralizedPose__feet_acc(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__get_const_function__GeneralizedPose__feet_acc(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__get_function__GeneralizedPose__feet_acc(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__fetch_function__GeneralizedPose__feet_acc(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__get_const_function__GeneralizedPose__feet_acc(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__assign_function__GeneralizedPose__feet_acc(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__get_function__GeneralizedPose__feet_acc(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__resize_function__GeneralizedPose__feet_acc(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__size_function__GeneralizedPose__feet_vel(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__get_const_function__GeneralizedPose__feet_vel(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__get_function__GeneralizedPose__feet_vel(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__fetch_function__GeneralizedPose__feet_vel(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__get_const_function__GeneralizedPose__feet_vel(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__assign_function__GeneralizedPose__feet_vel(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__get_function__GeneralizedPose__feet_vel(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__resize_function__GeneralizedPose__feet_vel(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__size_function__GeneralizedPose__feet_pos(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__get_const_function__GeneralizedPose__feet_pos(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__get_function__GeneralizedPose__feet_pos(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__fetch_function__GeneralizedPose__feet_pos(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__get_const_function__GeneralizedPose__feet_pos(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__assign_function__GeneralizedPose__feet_pos(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__get_function__GeneralizedPose__feet_pos(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__resize_function__GeneralizedPose__feet_pos(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__size_function__GeneralizedPose__contact_feet(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__get_const_function__GeneralizedPose__contact_feet(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__get_function__GeneralizedPose__contact_feet(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__fetch_function__GeneralizedPose__contact_feet(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__get_const_function__GeneralizedPose__contact_feet(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__assign_function__GeneralizedPose__contact_feet(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__get_function__GeneralizedPose__contact_feet(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__resize_function__GeneralizedPose__contact_feet(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__GeneralizedPose_message_member_array[9] = {
  {
    "base_acc",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(generalized_pose_msgs__msg__GeneralizedPose, base_acc),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "base_vel",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(generalized_pose_msgs__msg__GeneralizedPose, base_vel),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "base_pos",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(generalized_pose_msgs__msg__GeneralizedPose, base_pos),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "base_angvel",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(generalized_pose_msgs__msg__GeneralizedPose, base_angvel),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "base_quat",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(generalized_pose_msgs__msg__GeneralizedPose, base_quat),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feet_acc",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(generalized_pose_msgs__msg__GeneralizedPose, feet_acc),  // bytes offset in struct
    NULL,  // default value
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__size_function__GeneralizedPose__feet_acc,  // size() function pointer
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__get_const_function__GeneralizedPose__feet_acc,  // get_const(index) function pointer
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__get_function__GeneralizedPose__feet_acc,  // get(index) function pointer
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__fetch_function__GeneralizedPose__feet_acc,  // fetch(index, &value) function pointer
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__assign_function__GeneralizedPose__feet_acc,  // assign(index, value) function pointer
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__resize_function__GeneralizedPose__feet_acc  // resize(index) function pointer
  },
  {
    "feet_vel",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(generalized_pose_msgs__msg__GeneralizedPose, feet_vel),  // bytes offset in struct
    NULL,  // default value
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__size_function__GeneralizedPose__feet_vel,  // size() function pointer
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__get_const_function__GeneralizedPose__feet_vel,  // get_const(index) function pointer
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__get_function__GeneralizedPose__feet_vel,  // get(index) function pointer
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__fetch_function__GeneralizedPose__feet_vel,  // fetch(index, &value) function pointer
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__assign_function__GeneralizedPose__feet_vel,  // assign(index, value) function pointer
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__resize_function__GeneralizedPose__feet_vel  // resize(index) function pointer
  },
  {
    "feet_pos",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(generalized_pose_msgs__msg__GeneralizedPose, feet_pos),  // bytes offset in struct
    NULL,  // default value
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__size_function__GeneralizedPose__feet_pos,  // size() function pointer
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__get_const_function__GeneralizedPose__feet_pos,  // get_const(index) function pointer
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__get_function__GeneralizedPose__feet_pos,  // get(index) function pointer
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__fetch_function__GeneralizedPose__feet_pos,  // fetch(index, &value) function pointer
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__assign_function__GeneralizedPose__feet_pos,  // assign(index, value) function pointer
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__resize_function__GeneralizedPose__feet_pos  // resize(index) function pointer
  },
  {
    "contact_feet",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(generalized_pose_msgs__msg__GeneralizedPose, contact_feet),  // bytes offset in struct
    NULL,  // default value
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__size_function__GeneralizedPose__contact_feet,  // size() function pointer
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__get_const_function__GeneralizedPose__contact_feet,  // get_const(index) function pointer
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__get_function__GeneralizedPose__contact_feet,  // get(index) function pointer
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__fetch_function__GeneralizedPose__contact_feet,  // fetch(index, &value) function pointer
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__assign_function__GeneralizedPose__contact_feet,  // assign(index, value) function pointer
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__resize_function__GeneralizedPose__contact_feet  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__GeneralizedPose_message_members = {
  "generalized_pose_msgs__msg",  // message namespace
  "GeneralizedPose",  // message name
  9,  // number of fields
  sizeof(generalized_pose_msgs__msg__GeneralizedPose),
  generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__GeneralizedPose_message_member_array,  // message members
  generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__GeneralizedPose_init_function,  // function to initialize message memory (memory has to be allocated)
  generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__GeneralizedPose_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__GeneralizedPose_message_type_support_handle = {
  0,
  &generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__GeneralizedPose_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_generalized_pose_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, generalized_pose_msgs, msg, GeneralizedPose)() {
  generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__GeneralizedPose_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__GeneralizedPose_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__GeneralizedPose_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__GeneralizedPose_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__GeneralizedPose_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Quaternion)();
  if (!generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__GeneralizedPose_message_type_support_handle.typesupport_identifier) {
    generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__GeneralizedPose_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &generalized_pose_msgs__msg__GeneralizedPose__rosidl_typesupport_introspection_c__GeneralizedPose_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
