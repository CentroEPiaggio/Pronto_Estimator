// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from velocity_command_msgs:msg/SimpleVelocityCommand.idl
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
#include "velocity_command_msgs/msg/detail/simple_velocity_command__struct.h"
#include "velocity_command_msgs/msg/detail/simple_velocity_command__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool velocity_command_msgs__msg__simple_velocity_command__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[73];
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
    assert(strncmp("velocity_command_msgs.msg._simple_velocity_command.SimpleVelocityCommand", full_classname_dest, 72) == 0);
  }
  velocity_command_msgs__msg__SimpleVelocityCommand * ros_message = _ros_message;
  {  // velocity_forward
    PyObject * field = PyObject_GetAttrString(_pymsg, "velocity_forward");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->velocity_forward = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // velocity_lateral
    PyObject * field = PyObject_GetAttrString(_pymsg, "velocity_lateral");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->velocity_lateral = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // yaw_rate
    PyObject * field = PyObject_GetAttrString(_pymsg, "yaw_rate");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->yaw_rate = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * velocity_command_msgs__msg__simple_velocity_command__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SimpleVelocityCommand */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("velocity_command_msgs.msg._simple_velocity_command");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SimpleVelocityCommand");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  velocity_command_msgs__msg__SimpleVelocityCommand * ros_message = (velocity_command_msgs__msg__SimpleVelocityCommand *)raw_ros_message;
  {  // velocity_forward
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->velocity_forward);
    {
      int rc = PyObject_SetAttrString(_pymessage, "velocity_forward", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // velocity_lateral
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->velocity_lateral);
    {
      int rc = PyObject_SetAttrString(_pymessage, "velocity_lateral", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // yaw_rate
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->yaw_rate);
    {
      int rc = PyObject_SetAttrString(_pymessage, "yaw_rate", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}