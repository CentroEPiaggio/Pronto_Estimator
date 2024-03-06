// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from generalized_pose_msgs:msg/GeneralizedPoseWithTime.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "generalized_pose_msgs/msg/detail/generalized_pose_with_time__struct.h"
#include "generalized_pose_msgs/msg/detail/generalized_pose_with_time__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace generalized_pose_msgs
{

namespace msg
{

namespace rosidl_typesupport_c
{

typedef struct _GeneralizedPoseWithTime_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GeneralizedPoseWithTime_type_support_ids_t;

static const _GeneralizedPoseWithTime_type_support_ids_t _GeneralizedPoseWithTime_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _GeneralizedPoseWithTime_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GeneralizedPoseWithTime_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GeneralizedPoseWithTime_type_support_symbol_names_t _GeneralizedPoseWithTime_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, generalized_pose_msgs, msg, GeneralizedPoseWithTime)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, generalized_pose_msgs, msg, GeneralizedPoseWithTime)),
  }
};

typedef struct _GeneralizedPoseWithTime_type_support_data_t
{
  void * data[2];
} _GeneralizedPoseWithTime_type_support_data_t;

static _GeneralizedPoseWithTime_type_support_data_t _GeneralizedPoseWithTime_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GeneralizedPoseWithTime_message_typesupport_map = {
  2,
  "generalized_pose_msgs",
  &_GeneralizedPoseWithTime_message_typesupport_ids.typesupport_identifier[0],
  &_GeneralizedPoseWithTime_message_typesupport_symbol_names.symbol_name[0],
  &_GeneralizedPoseWithTime_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GeneralizedPoseWithTime_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GeneralizedPoseWithTime_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace msg

}  // namespace generalized_pose_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, generalized_pose_msgs, msg, GeneralizedPoseWithTime)() {
  return &::generalized_pose_msgs::msg::rosidl_typesupport_c::GeneralizedPoseWithTime_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
