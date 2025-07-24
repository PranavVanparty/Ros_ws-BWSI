// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from fizzbuzz_interfaces:msg/FizzBuzz.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "fizzbuzz_interfaces/msg/detail/fizz_buzz__rosidl_typesupport_introspection_c.h"
#include "fizzbuzz_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "fizzbuzz_interfaces/msg/detail/fizz_buzz__functions.h"
#include "fizzbuzz_interfaces/msg/detail/fizz_buzz__struct.h"


// Include directives for member types
// Member `fizzbuzz`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void fizzbuzz_interfaces__msg__FizzBuzz__rosidl_typesupport_introspection_c__FizzBuzz_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  fizzbuzz_interfaces__msg__FizzBuzz__init(message_memory);
}

void fizzbuzz_interfaces__msg__FizzBuzz__rosidl_typesupport_introspection_c__FizzBuzz_fini_function(void * message_memory)
{
  fizzbuzz_interfaces__msg__FizzBuzz__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember fizzbuzz_interfaces__msg__FizzBuzz__rosidl_typesupport_introspection_c__FizzBuzz_message_member_array[5] = {
  {
    "fizzbuzz",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(fizzbuzz_interfaces__msg__FizzBuzz, fizzbuzz),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "fizz_ratio",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(fizzbuzz_interfaces__msg__FizzBuzz, fizz_ratio),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "buzz_ratio",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(fizzbuzz_interfaces__msg__FizzBuzz, buzz_ratio),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "fizzbuzz_ratio",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(fizzbuzz_interfaces__msg__FizzBuzz, fizzbuzz_ratio),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "number_total",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(fizzbuzz_interfaces__msg__FizzBuzz, number_total),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers fizzbuzz_interfaces__msg__FizzBuzz__rosidl_typesupport_introspection_c__FizzBuzz_message_members = {
  "fizzbuzz_interfaces__msg",  // message namespace
  "FizzBuzz",  // message name
  5,  // number of fields
  sizeof(fizzbuzz_interfaces__msg__FizzBuzz),
  false,  // has_any_key_member_
  fizzbuzz_interfaces__msg__FizzBuzz__rosidl_typesupport_introspection_c__FizzBuzz_message_member_array,  // message members
  fizzbuzz_interfaces__msg__FizzBuzz__rosidl_typesupport_introspection_c__FizzBuzz_init_function,  // function to initialize message memory (memory has to be allocated)
  fizzbuzz_interfaces__msg__FizzBuzz__rosidl_typesupport_introspection_c__FizzBuzz_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t fizzbuzz_interfaces__msg__FizzBuzz__rosidl_typesupport_introspection_c__FizzBuzz_message_type_support_handle = {
  0,
  &fizzbuzz_interfaces__msg__FizzBuzz__rosidl_typesupport_introspection_c__FizzBuzz_message_members,
  get_message_typesupport_handle_function,
  &fizzbuzz_interfaces__msg__FizzBuzz__get_type_hash,
  &fizzbuzz_interfaces__msg__FizzBuzz__get_type_description,
  &fizzbuzz_interfaces__msg__FizzBuzz__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_fizzbuzz_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, fizzbuzz_interfaces, msg, FizzBuzz)() {
  if (!fizzbuzz_interfaces__msg__FizzBuzz__rosidl_typesupport_introspection_c__FizzBuzz_message_type_support_handle.typesupport_identifier) {
    fizzbuzz_interfaces__msg__FizzBuzz__rosidl_typesupport_introspection_c__FizzBuzz_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &fizzbuzz_interfaces__msg__FizzBuzz__rosidl_typesupport_introspection_c__FizzBuzz_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
