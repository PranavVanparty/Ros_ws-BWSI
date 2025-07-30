// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from fizzbuzz_interfaces:msg/FizzBuzz.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "fizzbuzz_interfaces/msg/fizz_buzz.h"


#ifndef FIZZBUZZ_INTERFACES__MSG__DETAIL__FIZZ_BUZZ__FUNCTIONS_H_
#define FIZZBUZZ_INTERFACES__MSG__DETAIL__FIZZ_BUZZ__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "fizzbuzz_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "fizzbuzz_interfaces/msg/detail/fizz_buzz__struct.h"

/// Initialize msg/FizzBuzz message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * fizzbuzz_interfaces__msg__FizzBuzz
 * )) before or use
 * fizzbuzz_interfaces__msg__FizzBuzz__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_fizzbuzz_interfaces
bool
fizzbuzz_interfaces__msg__FizzBuzz__init(fizzbuzz_interfaces__msg__FizzBuzz * msg);

/// Finalize msg/FizzBuzz message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_fizzbuzz_interfaces
void
fizzbuzz_interfaces__msg__FizzBuzz__fini(fizzbuzz_interfaces__msg__FizzBuzz * msg);

/// Create msg/FizzBuzz message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * fizzbuzz_interfaces__msg__FizzBuzz__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_fizzbuzz_interfaces
fizzbuzz_interfaces__msg__FizzBuzz *
fizzbuzz_interfaces__msg__FizzBuzz__create(void);

/// Destroy msg/FizzBuzz message.
/**
 * It calls
 * fizzbuzz_interfaces__msg__FizzBuzz__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_fizzbuzz_interfaces
void
fizzbuzz_interfaces__msg__FizzBuzz__destroy(fizzbuzz_interfaces__msg__FizzBuzz * msg);

/// Check for msg/FizzBuzz message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_fizzbuzz_interfaces
bool
fizzbuzz_interfaces__msg__FizzBuzz__are_equal(const fizzbuzz_interfaces__msg__FizzBuzz * lhs, const fizzbuzz_interfaces__msg__FizzBuzz * rhs);

/// Copy a msg/FizzBuzz message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_fizzbuzz_interfaces
bool
fizzbuzz_interfaces__msg__FizzBuzz__copy(
  const fizzbuzz_interfaces__msg__FizzBuzz * input,
  fizzbuzz_interfaces__msg__FizzBuzz * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_fizzbuzz_interfaces
const rosidl_type_hash_t *
fizzbuzz_interfaces__msg__FizzBuzz__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_fizzbuzz_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
fizzbuzz_interfaces__msg__FizzBuzz__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_fizzbuzz_interfaces
const rosidl_runtime_c__type_description__TypeSource *
fizzbuzz_interfaces__msg__FizzBuzz__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_fizzbuzz_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
fizzbuzz_interfaces__msg__FizzBuzz__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/FizzBuzz messages.
/**
 * It allocates the memory for the number of elements and calls
 * fizzbuzz_interfaces__msg__FizzBuzz__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_fizzbuzz_interfaces
bool
fizzbuzz_interfaces__msg__FizzBuzz__Sequence__init(fizzbuzz_interfaces__msg__FizzBuzz__Sequence * array, size_t size);

/// Finalize array of msg/FizzBuzz messages.
/**
 * It calls
 * fizzbuzz_interfaces__msg__FizzBuzz__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_fizzbuzz_interfaces
void
fizzbuzz_interfaces__msg__FizzBuzz__Sequence__fini(fizzbuzz_interfaces__msg__FizzBuzz__Sequence * array);

/// Create array of msg/FizzBuzz messages.
/**
 * It allocates the memory for the array and calls
 * fizzbuzz_interfaces__msg__FizzBuzz__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_fizzbuzz_interfaces
fizzbuzz_interfaces__msg__FizzBuzz__Sequence *
fizzbuzz_interfaces__msg__FizzBuzz__Sequence__create(size_t size);

/// Destroy array of msg/FizzBuzz messages.
/**
 * It calls
 * fizzbuzz_interfaces__msg__FizzBuzz__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_fizzbuzz_interfaces
void
fizzbuzz_interfaces__msg__FizzBuzz__Sequence__destroy(fizzbuzz_interfaces__msg__FizzBuzz__Sequence * array);

/// Check for msg/FizzBuzz message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_fizzbuzz_interfaces
bool
fizzbuzz_interfaces__msg__FizzBuzz__Sequence__are_equal(const fizzbuzz_interfaces__msg__FizzBuzz__Sequence * lhs, const fizzbuzz_interfaces__msg__FizzBuzz__Sequence * rhs);

/// Copy an array of msg/FizzBuzz messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_fizzbuzz_interfaces
bool
fizzbuzz_interfaces__msg__FizzBuzz__Sequence__copy(
  const fizzbuzz_interfaces__msg__FizzBuzz__Sequence * input,
  fizzbuzz_interfaces__msg__FizzBuzz__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // FIZZBUZZ_INTERFACES__MSG__DETAIL__FIZZ_BUZZ__FUNCTIONS_H_
