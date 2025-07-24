// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from fizzbuzz_interfaces:msg/FizzBuzz.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "fizzbuzz_interfaces/msg/detail/fizz_buzz__functions.h"
#include "fizzbuzz_interfaces/msg/detail/fizz_buzz__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace fizzbuzz_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void FizzBuzz_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) fizzbuzz_interfaces::msg::FizzBuzz(_init);
}

void FizzBuzz_fini_function(void * message_memory)
{
  auto typed_message = static_cast<fizzbuzz_interfaces::msg::FizzBuzz *>(message_memory);
  typed_message->~FizzBuzz();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember FizzBuzz_message_member_array[5] = {
  {
    "fizzbuzz",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(fizzbuzz_interfaces::msg::FizzBuzz, fizzbuzz),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "fizz_ratio",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(fizzbuzz_interfaces::msg::FizzBuzz, fizz_ratio),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "buzz_ratio",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(fizzbuzz_interfaces::msg::FizzBuzz, buzz_ratio),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "fizzbuzz_ratio",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(fizzbuzz_interfaces::msg::FizzBuzz, fizzbuzz_ratio),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "number_total",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(fizzbuzz_interfaces::msg::FizzBuzz, number_total),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers FizzBuzz_message_members = {
  "fizzbuzz_interfaces::msg",  // message namespace
  "FizzBuzz",  // message name
  5,  // number of fields
  sizeof(fizzbuzz_interfaces::msg::FizzBuzz),
  false,  // has_any_key_member_
  FizzBuzz_message_member_array,  // message members
  FizzBuzz_init_function,  // function to initialize message memory (memory has to be allocated)
  FizzBuzz_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t FizzBuzz_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &FizzBuzz_message_members,
  get_message_typesupport_handle_function,
  &fizzbuzz_interfaces__msg__FizzBuzz__get_type_hash,
  &fizzbuzz_interfaces__msg__FizzBuzz__get_type_description,
  &fizzbuzz_interfaces__msg__FizzBuzz__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace fizzbuzz_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<fizzbuzz_interfaces::msg::FizzBuzz>()
{
  return &::fizzbuzz_interfaces::msg::rosidl_typesupport_introspection_cpp::FizzBuzz_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, fizzbuzz_interfaces, msg, FizzBuzz)() {
  return &::fizzbuzz_interfaces::msg::rosidl_typesupport_introspection_cpp::FizzBuzz_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
