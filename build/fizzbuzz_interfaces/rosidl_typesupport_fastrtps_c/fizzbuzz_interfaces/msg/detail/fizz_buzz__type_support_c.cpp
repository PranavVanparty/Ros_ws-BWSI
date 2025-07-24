// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from fizzbuzz_interfaces:msg/FizzBuzz.idl
// generated code does not contain a copyright notice
#include "fizzbuzz_interfaces/msg/detail/fizz_buzz__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "fizzbuzz_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "fizzbuzz_interfaces/msg/detail/fizz_buzz__struct.h"
#include "fizzbuzz_interfaces/msg/detail/fizz_buzz__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // fizzbuzz
#include "rosidl_runtime_c/string_functions.h"  // fizzbuzz

// forward declare type support functions


using _FizzBuzz__ros_msg_type = fizzbuzz_interfaces__msg__FizzBuzz;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_fizzbuzz_interfaces
bool cdr_serialize_fizzbuzz_interfaces__msg__FizzBuzz(
  const fizzbuzz_interfaces__msg__FizzBuzz * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: fizzbuzz
  {
    const rosidl_runtime_c__String * str = &ros_message->fizzbuzz;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: fizz_ratio
  {
    cdr << ros_message->fizz_ratio;
  }

  // Field name: buzz_ratio
  {
    cdr << ros_message->buzz_ratio;
  }

  // Field name: fizzbuzz_ratio
  {
    cdr << ros_message->fizzbuzz_ratio;
  }

  // Field name: number_total
  {
    cdr << ros_message->number_total;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_fizzbuzz_interfaces
bool cdr_deserialize_fizzbuzz_interfaces__msg__FizzBuzz(
  eprosima::fastcdr::Cdr & cdr,
  fizzbuzz_interfaces__msg__FizzBuzz * ros_message)
{
  // Field name: fizzbuzz
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->fizzbuzz.data) {
      rosidl_runtime_c__String__init(&ros_message->fizzbuzz);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->fizzbuzz,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'fizzbuzz'\n");
      return false;
    }
  }

  // Field name: fizz_ratio
  {
    cdr >> ros_message->fizz_ratio;
  }

  // Field name: buzz_ratio
  {
    cdr >> ros_message->buzz_ratio;
  }

  // Field name: fizzbuzz_ratio
  {
    cdr >> ros_message->fizzbuzz_ratio;
  }

  // Field name: number_total
  {
    cdr >> ros_message->number_total;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_fizzbuzz_interfaces
size_t get_serialized_size_fizzbuzz_interfaces__msg__FizzBuzz(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _FizzBuzz__ros_msg_type * ros_message = static_cast<const _FizzBuzz__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: fizzbuzz
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->fizzbuzz.size + 1);

  // Field name: fizz_ratio
  {
    size_t item_size = sizeof(ros_message->fizz_ratio);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: buzz_ratio
  {
    size_t item_size = sizeof(ros_message->buzz_ratio);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: fizzbuzz_ratio
  {
    size_t item_size = sizeof(ros_message->fizzbuzz_ratio);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: number_total
  {
    size_t item_size = sizeof(ros_message->number_total);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_fizzbuzz_interfaces
size_t max_serialized_size_fizzbuzz_interfaces__msg__FizzBuzz(
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

  // Field name: fizzbuzz
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

  // Field name: fizz_ratio
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: buzz_ratio
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: fizzbuzz_ratio
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: number_total
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
    using DataType = fizzbuzz_interfaces__msg__FizzBuzz;
    is_plain =
      (
      offsetof(DataType, number_total) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_fizzbuzz_interfaces
bool cdr_serialize_key_fizzbuzz_interfaces__msg__FizzBuzz(
  const fizzbuzz_interfaces__msg__FizzBuzz * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: fizzbuzz
  {
    const rosidl_runtime_c__String * str = &ros_message->fizzbuzz;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: fizz_ratio
  {
    cdr << ros_message->fizz_ratio;
  }

  // Field name: buzz_ratio
  {
    cdr << ros_message->buzz_ratio;
  }

  // Field name: fizzbuzz_ratio
  {
    cdr << ros_message->fizzbuzz_ratio;
  }

  // Field name: number_total
  {
    cdr << ros_message->number_total;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_fizzbuzz_interfaces
size_t get_serialized_size_key_fizzbuzz_interfaces__msg__FizzBuzz(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _FizzBuzz__ros_msg_type * ros_message = static_cast<const _FizzBuzz__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: fizzbuzz
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->fizzbuzz.size + 1);

  // Field name: fizz_ratio
  {
    size_t item_size = sizeof(ros_message->fizz_ratio);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: buzz_ratio
  {
    size_t item_size = sizeof(ros_message->buzz_ratio);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: fizzbuzz_ratio
  {
    size_t item_size = sizeof(ros_message->fizzbuzz_ratio);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: number_total
  {
    size_t item_size = sizeof(ros_message->number_total);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_fizzbuzz_interfaces
size_t max_serialized_size_key_fizzbuzz_interfaces__msg__FizzBuzz(
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
  // Field name: fizzbuzz
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

  // Field name: fizz_ratio
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: buzz_ratio
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: fizzbuzz_ratio
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: number_total
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
    using DataType = fizzbuzz_interfaces__msg__FizzBuzz;
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
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const fizzbuzz_interfaces__msg__FizzBuzz * ros_message = static_cast<const fizzbuzz_interfaces__msg__FizzBuzz *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_fizzbuzz_interfaces__msg__FizzBuzz(ros_message, cdr);
}

static bool _FizzBuzz__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  fizzbuzz_interfaces__msg__FizzBuzz * ros_message = static_cast<fizzbuzz_interfaces__msg__FizzBuzz *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_fizzbuzz_interfaces__msg__FizzBuzz(cdr, ros_message);
}

static uint32_t _FizzBuzz__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_fizzbuzz_interfaces__msg__FizzBuzz(
      untyped_ros_message, 0));
}

static size_t _FizzBuzz__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_fizzbuzz_interfaces__msg__FizzBuzz(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_FizzBuzz = {
  "fizzbuzz_interfaces::msg",
  "FizzBuzz",
  _FizzBuzz__cdr_serialize,
  _FizzBuzz__cdr_deserialize,
  _FizzBuzz__get_serialized_size,
  _FizzBuzz__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _FizzBuzz__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_FizzBuzz,
  get_message_typesupport_handle_function,
  &fizzbuzz_interfaces__msg__FizzBuzz__get_type_hash,
  &fizzbuzz_interfaces__msg__FizzBuzz__get_type_description,
  &fizzbuzz_interfaces__msg__FizzBuzz__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, fizzbuzz_interfaces, msg, FizzBuzz)() {
  return &_FizzBuzz__type_support;
}

#if defined(__cplusplus)
}
#endif
