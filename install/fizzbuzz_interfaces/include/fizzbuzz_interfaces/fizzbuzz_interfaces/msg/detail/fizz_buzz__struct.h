// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from fizzbuzz_interfaces:msg/FizzBuzz.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "fizzbuzz_interfaces/msg/fizz_buzz.h"


#ifndef FIZZBUZZ_INTERFACES__MSG__DETAIL__FIZZ_BUZZ__STRUCT_H_
#define FIZZBUZZ_INTERFACES__MSG__DETAIL__FIZZ_BUZZ__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'fizzbuzz'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/FizzBuzz in the package fizzbuzz_interfaces.
typedef struct fizzbuzz_interfaces__msg__FizzBuzz
{
  /// the result of the fizzbuzz function
  rosidl_runtime_c__String fizzbuzz;
  /// the ratio of fizz results to non fizz results
  float fizz_ratio;
  /// the ratio of buzz results to non buzz results
  float buzz_ratio;
  /// the ratio of fizzbuzz results to non fizzbuzz results
  float fizzbuzz_ratio;
  /// the total number of numbers received.
  int32_t number_total;
} fizzbuzz_interfaces__msg__FizzBuzz;

// Struct for a sequence of fizzbuzz_interfaces__msg__FizzBuzz.
typedef struct fizzbuzz_interfaces__msg__FizzBuzz__Sequence
{
  fizzbuzz_interfaces__msg__FizzBuzz * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} fizzbuzz_interfaces__msg__FizzBuzz__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // FIZZBUZZ_INTERFACES__MSG__DETAIL__FIZZ_BUZZ__STRUCT_H_
