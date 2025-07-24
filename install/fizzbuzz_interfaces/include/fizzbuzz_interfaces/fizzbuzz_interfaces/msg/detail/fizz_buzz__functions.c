// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from fizzbuzz_interfaces:msg/FizzBuzz.idl
// generated code does not contain a copyright notice
#include "fizzbuzz_interfaces/msg/detail/fizz_buzz__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `fizzbuzz`
#include "rosidl_runtime_c/string_functions.h"

bool
fizzbuzz_interfaces__msg__FizzBuzz__init(fizzbuzz_interfaces__msg__FizzBuzz * msg)
{
  if (!msg) {
    return false;
  }
  // fizzbuzz
  if (!rosidl_runtime_c__String__init(&msg->fizzbuzz)) {
    fizzbuzz_interfaces__msg__FizzBuzz__fini(msg);
    return false;
  }
  // fizz_ratio
  // buzz_ratio
  // fizzbuzz_ratio
  // number_total
  return true;
}

void
fizzbuzz_interfaces__msg__FizzBuzz__fini(fizzbuzz_interfaces__msg__FizzBuzz * msg)
{
  if (!msg) {
    return;
  }
  // fizzbuzz
  rosidl_runtime_c__String__fini(&msg->fizzbuzz);
  // fizz_ratio
  // buzz_ratio
  // fizzbuzz_ratio
  // number_total
}

bool
fizzbuzz_interfaces__msg__FizzBuzz__are_equal(const fizzbuzz_interfaces__msg__FizzBuzz * lhs, const fizzbuzz_interfaces__msg__FizzBuzz * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // fizzbuzz
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->fizzbuzz), &(rhs->fizzbuzz)))
  {
    return false;
  }
  // fizz_ratio
  if (lhs->fizz_ratio != rhs->fizz_ratio) {
    return false;
  }
  // buzz_ratio
  if (lhs->buzz_ratio != rhs->buzz_ratio) {
    return false;
  }
  // fizzbuzz_ratio
  if (lhs->fizzbuzz_ratio != rhs->fizzbuzz_ratio) {
    return false;
  }
  // number_total
  if (lhs->number_total != rhs->number_total) {
    return false;
  }
  return true;
}

bool
fizzbuzz_interfaces__msg__FizzBuzz__copy(
  const fizzbuzz_interfaces__msg__FizzBuzz * input,
  fizzbuzz_interfaces__msg__FizzBuzz * output)
{
  if (!input || !output) {
    return false;
  }
  // fizzbuzz
  if (!rosidl_runtime_c__String__copy(
      &(input->fizzbuzz), &(output->fizzbuzz)))
  {
    return false;
  }
  // fizz_ratio
  output->fizz_ratio = input->fizz_ratio;
  // buzz_ratio
  output->buzz_ratio = input->buzz_ratio;
  // fizzbuzz_ratio
  output->fizzbuzz_ratio = input->fizzbuzz_ratio;
  // number_total
  output->number_total = input->number_total;
  return true;
}

fizzbuzz_interfaces__msg__FizzBuzz *
fizzbuzz_interfaces__msg__FizzBuzz__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  fizzbuzz_interfaces__msg__FizzBuzz * msg = (fizzbuzz_interfaces__msg__FizzBuzz *)allocator.allocate(sizeof(fizzbuzz_interfaces__msg__FizzBuzz), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(fizzbuzz_interfaces__msg__FizzBuzz));
  bool success = fizzbuzz_interfaces__msg__FizzBuzz__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
fizzbuzz_interfaces__msg__FizzBuzz__destroy(fizzbuzz_interfaces__msg__FizzBuzz * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    fizzbuzz_interfaces__msg__FizzBuzz__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
fizzbuzz_interfaces__msg__FizzBuzz__Sequence__init(fizzbuzz_interfaces__msg__FizzBuzz__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  fizzbuzz_interfaces__msg__FizzBuzz * data = NULL;

  if (size) {
    data = (fizzbuzz_interfaces__msg__FizzBuzz *)allocator.zero_allocate(size, sizeof(fizzbuzz_interfaces__msg__FizzBuzz), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = fizzbuzz_interfaces__msg__FizzBuzz__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        fizzbuzz_interfaces__msg__FizzBuzz__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
fizzbuzz_interfaces__msg__FizzBuzz__Sequence__fini(fizzbuzz_interfaces__msg__FizzBuzz__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      fizzbuzz_interfaces__msg__FizzBuzz__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

fizzbuzz_interfaces__msg__FizzBuzz__Sequence *
fizzbuzz_interfaces__msg__FizzBuzz__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  fizzbuzz_interfaces__msg__FizzBuzz__Sequence * array = (fizzbuzz_interfaces__msg__FizzBuzz__Sequence *)allocator.allocate(sizeof(fizzbuzz_interfaces__msg__FizzBuzz__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = fizzbuzz_interfaces__msg__FizzBuzz__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
fizzbuzz_interfaces__msg__FizzBuzz__Sequence__destroy(fizzbuzz_interfaces__msg__FizzBuzz__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    fizzbuzz_interfaces__msg__FizzBuzz__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
fizzbuzz_interfaces__msg__FizzBuzz__Sequence__are_equal(const fizzbuzz_interfaces__msg__FizzBuzz__Sequence * lhs, const fizzbuzz_interfaces__msg__FizzBuzz__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!fizzbuzz_interfaces__msg__FizzBuzz__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
fizzbuzz_interfaces__msg__FizzBuzz__Sequence__copy(
  const fizzbuzz_interfaces__msg__FizzBuzz__Sequence * input,
  fizzbuzz_interfaces__msg__FizzBuzz__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(fizzbuzz_interfaces__msg__FizzBuzz);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    fizzbuzz_interfaces__msg__FizzBuzz * data =
      (fizzbuzz_interfaces__msg__FizzBuzz *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!fizzbuzz_interfaces__msg__FizzBuzz__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          fizzbuzz_interfaces__msg__FizzBuzz__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!fizzbuzz_interfaces__msg__FizzBuzz__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
