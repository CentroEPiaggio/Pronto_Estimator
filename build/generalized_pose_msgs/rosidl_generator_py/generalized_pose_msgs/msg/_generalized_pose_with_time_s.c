// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from generalized_pose_msgs:msg/GeneralizedPoseWithTime.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "generalized_pose_msgs/msg/detail/generalized_pose_with_time__struct.h"
#include "generalized_pose_msgs/msg/detail/generalized_pose_with_time__functions.h"

bool generalized_pose_msgs__msg__generalized_pose__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * generalized_pose_msgs__msg__generalized_pose__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool generalized_pose_msgs__msg__generalized_pose_with_time__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[78];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("generalized_pose_msgs.msg._generalized_pose_with_time.GeneralizedPoseWithTime", full_classname_dest, 77) == 0);
  }
  generalized_pose_msgs__msg__GeneralizedPoseWithTime * ros_message = _ros_message;
  {  // generalized_pose
    PyObject * field = PyObject_GetAttrString(_pymsg, "generalized_pose");
    if (!field) {
      return false;
    }
    if (!generalized_pose_msgs__msg__generalized_pose__convert_from_py(field, &ros_message->generalized_pose)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // time
    PyObject * field = PyObject_GetAttrString(_pymsg, "time");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->time = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * generalized_pose_msgs__msg__generalized_pose_with_time__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GeneralizedPoseWithTime */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("generalized_pose_msgs.msg._generalized_pose_with_time");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GeneralizedPoseWithTime");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  generalized_pose_msgs__msg__GeneralizedPoseWithTime * ros_message = (generalized_pose_msgs__msg__GeneralizedPoseWithTime *)raw_ros_message;
  {  // generalized_pose
    PyObject * field = NULL;
    field = generalized_pose_msgs__msg__generalized_pose__convert_to_py(&ros_message->generalized_pose);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "generalized_pose", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // time
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->time);
    {
      int rc = PyObject_SetAttrString(_pymessage, "time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
