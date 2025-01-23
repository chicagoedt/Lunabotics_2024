// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from interfaces_pkg:msg/DutyCycleOutput.idl
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
#include "interfaces_pkg/msg/detail/duty_cycle_output__struct.h"
#include "interfaces_pkg/msg/detail/duty_cycle_output__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool interfaces_pkg__msg__duty_cycle_output__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[54];
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
    assert(strncmp("interfaces_pkg.msg._duty_cycle_output.DutyCycleOutput", full_classname_dest, 53) == 0);
  }
  interfaces_pkg__msg__DutyCycleOutput * ros_message = _ros_message;
  {  // left_drive_output
    PyObject * field = PyObject_GetAttrString(_pymsg, "left_drive_output");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->left_drive_output = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // right_drive_output
    PyObject * field = PyObject_GetAttrString(_pymsg, "right_drive_output");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->right_drive_output = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // left_lift_output
    PyObject * field = PyObject_GetAttrString(_pymsg, "left_lift_output");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->left_lift_output = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // right_lift_output
    PyObject * field = PyObject_GetAttrString(_pymsg, "right_lift_output");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->right_lift_output = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // tilt_output
    PyObject * field = PyObject_GetAttrString(_pymsg, "tilt_output");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->tilt_output = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vibrator_output
    PyObject * field = PyObject_GetAttrString(_pymsg, "vibrator_output");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vibrator_output = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * interfaces_pkg__msg__duty_cycle_output__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of DutyCycleOutput */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("interfaces_pkg.msg._duty_cycle_output");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "DutyCycleOutput");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  interfaces_pkg__msg__DutyCycleOutput * ros_message = (interfaces_pkg__msg__DutyCycleOutput *)raw_ros_message;
  {  // left_drive_output
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->left_drive_output);
    {
      int rc = PyObject_SetAttrString(_pymessage, "left_drive_output", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // right_drive_output
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->right_drive_output);
    {
      int rc = PyObject_SetAttrString(_pymessage, "right_drive_output", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // left_lift_output
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->left_lift_output);
    {
      int rc = PyObject_SetAttrString(_pymessage, "left_lift_output", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // right_lift_output
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->right_lift_output);
    {
      int rc = PyObject_SetAttrString(_pymessage, "right_lift_output", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tilt_output
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->tilt_output);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tilt_output", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vibrator_output
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vibrator_output);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vibrator_output", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
