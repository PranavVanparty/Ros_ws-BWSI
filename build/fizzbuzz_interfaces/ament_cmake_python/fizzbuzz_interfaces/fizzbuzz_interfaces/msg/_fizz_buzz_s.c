// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from fizzbuzz_interfaces:msg/FizzBuzz.idl
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
#include "fizzbuzz_interfaces/msg/detail/fizz_buzz__struct.h"
#include "fizzbuzz_interfaces/msg/detail/fizz_buzz__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool fizzbuzz_interfaces__msg__fizz_buzz__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[44];
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
    assert(strncmp("fizzbuzz_interfaces.msg._fizz_buzz.FizzBuzz", full_classname_dest, 43) == 0);
  }
  fizzbuzz_interfaces__msg__FizzBuzz * ros_message = _ros_message;
  {  // fizzbuzz
    PyObject * field = PyObject_GetAttrString(_pymsg, "fizzbuzz");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->fizzbuzz, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // fizz_ratio
    PyObject * field = PyObject_GetAttrString(_pymsg, "fizz_ratio");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fizz_ratio = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // buzz_ratio
    PyObject * field = PyObject_GetAttrString(_pymsg, "buzz_ratio");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->buzz_ratio = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fizzbuzz_ratio
    PyObject * field = PyObject_GetAttrString(_pymsg, "fizzbuzz_ratio");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fizzbuzz_ratio = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // number_total
    PyObject * field = PyObject_GetAttrString(_pymsg, "number_total");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->number_total = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * fizzbuzz_interfaces__msg__fizz_buzz__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of FizzBuzz */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("fizzbuzz_interfaces.msg._fizz_buzz");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "FizzBuzz");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  fizzbuzz_interfaces__msg__FizzBuzz * ros_message = (fizzbuzz_interfaces__msg__FizzBuzz *)raw_ros_message;
  {  // fizzbuzz
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->fizzbuzz.data,
      strlen(ros_message->fizzbuzz.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "fizzbuzz", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fizz_ratio
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fizz_ratio);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fizz_ratio", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // buzz_ratio
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->buzz_ratio);
    {
      int rc = PyObject_SetAttrString(_pymessage, "buzz_ratio", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fizzbuzz_ratio
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fizzbuzz_ratio);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fizzbuzz_ratio", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // number_total
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->number_total);
    {
      int rc = PyObject_SetAttrString(_pymessage, "number_total", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
