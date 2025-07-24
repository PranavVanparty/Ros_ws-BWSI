// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from fizzbuzz_interfaces:msg/FizzBuzz.idl
// generated code does not contain a copyright notice

#include "fizzbuzz_interfaces/msg/detail/fizz_buzz__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_fizzbuzz_interfaces
const rosidl_type_hash_t *
fizzbuzz_interfaces__msg__FizzBuzz__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x31, 0xf2, 0x0f, 0x56, 0x19, 0x3f, 0x2c, 0xae,
      0x65, 0x8b, 0x9a, 0xa4, 0x68, 0x9a, 0xeb, 0x4f,
      0xdd, 0xa0, 0xf2, 0xd8, 0x2b, 0x56, 0x9f, 0xc5,
      0x4d, 0xb7, 0x28, 0x18, 0x36, 0x44, 0xb1, 0xcf,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char fizzbuzz_interfaces__msg__FizzBuzz__TYPE_NAME[] = "fizzbuzz_interfaces/msg/FizzBuzz";

// Define type names, field names, and default values
static char fizzbuzz_interfaces__msg__FizzBuzz__FIELD_NAME__fizzbuzz[] = "fizzbuzz";
static char fizzbuzz_interfaces__msg__FizzBuzz__FIELD_NAME__fizz_ratio[] = "fizz_ratio";
static char fizzbuzz_interfaces__msg__FizzBuzz__FIELD_NAME__buzz_ratio[] = "buzz_ratio";
static char fizzbuzz_interfaces__msg__FizzBuzz__FIELD_NAME__fizzbuzz_ratio[] = "fizzbuzz_ratio";
static char fizzbuzz_interfaces__msg__FizzBuzz__FIELD_NAME__number_total[] = "number_total";

static rosidl_runtime_c__type_description__Field fizzbuzz_interfaces__msg__FizzBuzz__FIELDS[] = {
  {
    {fizzbuzz_interfaces__msg__FizzBuzz__FIELD_NAME__fizzbuzz, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {fizzbuzz_interfaces__msg__FizzBuzz__FIELD_NAME__fizz_ratio, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {fizzbuzz_interfaces__msg__FizzBuzz__FIELD_NAME__buzz_ratio, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {fizzbuzz_interfaces__msg__FizzBuzz__FIELD_NAME__fizzbuzz_ratio, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {fizzbuzz_interfaces__msg__FizzBuzz__FIELD_NAME__number_total, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
fizzbuzz_interfaces__msg__FizzBuzz__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {fizzbuzz_interfaces__msg__FizzBuzz__TYPE_NAME, 32, 32},
      {fizzbuzz_interfaces__msg__FizzBuzz__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string fizzbuzz  # the result of the fizzbuzz function\n"
  "float32 fizz_ratio  # the ratio of fizz results to non fizz results\n"
  "float32 buzz_ratio  # the ratio of buzz results to non buzz results\n"
  "float32 fizzbuzz_ratio  # the ratio of fizzbuzz results to non fizzbuzz results\n"
  "int32 number_total  # the total number of numbers received.";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
fizzbuzz_interfaces__msg__FizzBuzz__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {fizzbuzz_interfaces__msg__FizzBuzz__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 330, 330},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
fizzbuzz_interfaces__msg__FizzBuzz__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *fizzbuzz_interfaces__msg__FizzBuzz__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
