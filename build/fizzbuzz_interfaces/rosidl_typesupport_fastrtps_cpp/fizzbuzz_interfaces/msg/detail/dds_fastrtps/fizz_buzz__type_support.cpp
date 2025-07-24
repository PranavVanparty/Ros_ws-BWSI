// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from fizzbuzz_interfaces:msg/FizzBuzz.idl
// generated code does not contain a copyright notice
#include "fizzbuzz_interfaces/msg/detail/fizz_buzz__rosidl_typesupport_fastrtps_cpp.hpp"
#include "fizzbuzz_interfaces/msg/detail/fizz_buzz__functions.h"
#include "fizzbuzz_interfaces/msg/detail/fizz_buzz__struct.hpp"

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace fizzbuzz_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{


bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fizzbuzz_interfaces
cdr_serialize(
  const fizzbuzz_interfaces::msg::FizzBuzz & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: fizzbuzz
  cdr << ros_message.fizzbuzz;

  // Member: fizz_ratio
  cdr << ros_message.fizz_ratio;

  // Member: buzz_ratio
  cdr << ros_message.buzz_ratio;

  // Member: fizzbuzz_ratio
  cdr << ros_message.fizzbuzz_ratio;

  // Member: number_total
  cdr << ros_message.number_total;

  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fizzbuzz_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  fizzbuzz_interfaces::msg::FizzBuzz & ros_message)
{
  // Member: fizzbuzz
  cdr >> ros_message.fizzbuzz;

  // Member: fizz_ratio
  cdr >> ros_message.fizz_ratio;

  // Member: buzz_ratio
  cdr >> ros_message.buzz_ratio;

  // Member: fizzbuzz_ratio
  cdr >> ros_message.fizzbuzz_ratio;

  // Member: number_total
  cdr >> ros_message.number_total;

  return true;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fizzbuzz_interfaces
get_serialized_size(
  const fizzbuzz_interfaces::msg::FizzBuzz & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: fizzbuzz
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.fizzbuzz.size() + 1);

  // Member: fizz_ratio
  {
    size_t item_size = sizeof(ros_message.fizz_ratio);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: buzz_ratio
  {
    size_t item_size = sizeof(ros_message.buzz_ratio);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: fizzbuzz_ratio
  {
    size_t item_size = sizeof(ros_message.fizzbuzz_ratio);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: number_total
  {
    size_t item_size = sizeof(ros_message.number_total);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fizzbuzz_interfaces
max_serialized_size_FizzBuzz(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Member: fizzbuzz
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // Member: fizz_ratio
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: buzz_ratio
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: fizzbuzz_ratio
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: number_total
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = fizzbuzz_interfaces::msg::FizzBuzz;
    is_plain =
      (
      offsetof(DataType, number_total) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fizzbuzz_interfaces
cdr_serialize_key(
  const fizzbuzz_interfaces::msg::FizzBuzz & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: fizzbuzz
  cdr << ros_message.fizzbuzz;

  // Member: fizz_ratio
  cdr << ros_message.fizz_ratio;

  // Member: buzz_ratio
  cdr << ros_message.buzz_ratio;

  // Member: fizzbuzz_ratio
  cdr << ros_message.fizzbuzz_ratio;

  // Member: number_total
  cdr << ros_message.number_total;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fizzbuzz_interfaces
get_serialized_size_key(
  const fizzbuzz_interfaces::msg::FizzBuzz & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: fizzbuzz
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.fizzbuzz.size() + 1);

  // Member: fizz_ratio
  {
    size_t item_size = sizeof(ros_message.fizz_ratio);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: buzz_ratio
  {
    size_t item_size = sizeof(ros_message.buzz_ratio);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: fizzbuzz_ratio
  {
    size_t item_size = sizeof(ros_message.fizzbuzz_ratio);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: number_total
  {
    size_t item_size = sizeof(ros_message.number_total);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fizzbuzz_interfaces
max_serialized_size_key_FizzBuzz(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Member: fizzbuzz
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: fizz_ratio
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: buzz_ratio
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: fizzbuzz_ratio
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: number_total
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = fizzbuzz_interfaces::msg::FizzBuzz;
    is_plain =
      (
      offsetof(DataType, number_total) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}


static bool _FizzBuzz__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const fizzbuzz_interfaces::msg::FizzBuzz *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _FizzBuzz__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<fizzbuzz_interfaces::msg::FizzBuzz *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _FizzBuzz__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const fizzbuzz_interfaces::msg::FizzBuzz *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _FizzBuzz__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_FizzBuzz(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _FizzBuzz__callbacks = {
  "fizzbuzz_interfaces::msg",
  "FizzBuzz",
  _FizzBuzz__cdr_serialize,
  _FizzBuzz__cdr_deserialize,
  _FizzBuzz__get_serialized_size,
  _FizzBuzz__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _FizzBuzz__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_FizzBuzz__callbacks,
  get_message_typesupport_handle_function,
  &fizzbuzz_interfaces__msg__FizzBuzz__get_type_hash,
  &fizzbuzz_interfaces__msg__FizzBuzz__get_type_description,
  &fizzbuzz_interfaces__msg__FizzBuzz__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace fizzbuzz_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_fizzbuzz_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<fizzbuzz_interfaces::msg::FizzBuzz>()
{
  return &fizzbuzz_interfaces::msg::typesupport_fastrtps_cpp::_FizzBuzz__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, fizzbuzz_interfaces, msg, FizzBuzz)() {
  return &fizzbuzz_interfaces::msg::typesupport_fastrtps_cpp::_FizzBuzz__handle;
}

#ifdef __cplusplus
}
#endif
