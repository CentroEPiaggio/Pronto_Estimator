// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from generalized_pose_msgs:msg/GeneralizedPosesWithTime.idl
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
#include "generalized_pose_msgs/msg/detail/generalized_poses_with_time__struct.h"
#include "generalized_pose_msgs/msg/detail/generalized_poses_with_time__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "generalized_pose_msgs/msg/detail/generalized_pose_with_time__functions.h"
// end nested array functions include
bool generalized_pose_msgs__msg__generalized_pose_with_time__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * generalized_pose_msgs__msg__generalized_pose_with_time__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool generalized_pose_msgs__msg__generalized_poses_with_time__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[80];
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
    assert(strncmp("generalized_pose_msgs.msg._generalized_poses_with_time.GeneralizedPosesWithTime", full_classname_dest, 79) == 0);
  }
  generalized_pose_msgs__msg__GeneralizedPosesWithTime * ros_message = _ros_message;
  {  // generalized_poses_with_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "generalized_poses_with_time");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'generalized_poses_with_time'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence__init(&(ros_message->generalized_poses_with_time), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create generalized_pose_msgs__msg__GeneralizedPoseWithTime__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    generalized_pose_msgs__msg__GeneralizedPoseWithTime * dest = ros_message->generalized_poses_with_time.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!generalized_pose_msgs__msg__generalized_pose_with_time__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * generalized_pose_msgs__msg__generalized_poses_with_time__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GeneralizedPosesWithTime */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("generalized_pose_msgs.msg._generalized_poses_with_time");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GeneralizedPosesWithTime");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  generalized_pose_msgs__msg__GeneralizedPosesWithTime * ros_message = (generalized_pose_msgs__msg__GeneralizedPosesWithTime *)raw_ros_message;
  {  // generalized_poses_with_time
    PyObject * field = NULL;
    size_t size = ros_message->generalized_poses_with_time.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    generalized_pose_msgs__msg__GeneralizedPoseWithTime * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->generalized_poses_with_time.data[i]);
      PyObject * pyitem = generalized_pose_msgs__msg__generalized_pose_with_time__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "generalized_poses_with_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
