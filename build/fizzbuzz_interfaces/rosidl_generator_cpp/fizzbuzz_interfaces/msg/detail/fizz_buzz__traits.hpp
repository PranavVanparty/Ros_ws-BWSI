// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from fizzbuzz_interfaces:msg/FizzBuzz.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "fizzbuzz_interfaces/msg/fizz_buzz.hpp"


#ifndef FIZZBUZZ_INTERFACES__MSG__DETAIL__FIZZ_BUZZ__TRAITS_HPP_
#define FIZZBUZZ_INTERFACES__MSG__DETAIL__FIZZ_BUZZ__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "fizzbuzz_interfaces/msg/detail/fizz_buzz__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace fizzbuzz_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const FizzBuzz & msg,
  std::ostream & out)
{
  out << "{";
  // member: fizzbuzz
  {
    out << "fizzbuzz: ";
    rosidl_generator_traits::value_to_yaml(msg.fizzbuzz, out);
    out << ", ";
  }

  // member: fizz_ratio
  {
    out << "fizz_ratio: ";
    rosidl_generator_traits::value_to_yaml(msg.fizz_ratio, out);
    out << ", ";
  }

  // member: buzz_ratio
  {
    out << "buzz_ratio: ";
    rosidl_generator_traits::value_to_yaml(msg.buzz_ratio, out);
    out << ", ";
  }

  // member: fizzbuzz_ratio
  {
    out << "fizzbuzz_ratio: ";
    rosidl_generator_traits::value_to_yaml(msg.fizzbuzz_ratio, out);
    out << ", ";
  }

  // member: number_total
  {
    out << "number_total: ";
    rosidl_generator_traits::value_to_yaml(msg.number_total, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FizzBuzz & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: fizzbuzz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fizzbuzz: ";
    rosidl_generator_traits::value_to_yaml(msg.fizzbuzz, out);
    out << "\n";
  }

  // member: fizz_ratio
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fizz_ratio: ";
    rosidl_generator_traits::value_to_yaml(msg.fizz_ratio, out);
    out << "\n";
  }

  // member: buzz_ratio
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "buzz_ratio: ";
    rosidl_generator_traits::value_to_yaml(msg.buzz_ratio, out);
    out << "\n";
  }

  // member: fizzbuzz_ratio
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fizzbuzz_ratio: ";
    rosidl_generator_traits::value_to_yaml(msg.fizzbuzz_ratio, out);
    out << "\n";
  }

  // member: number_total
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "number_total: ";
    rosidl_generator_traits::value_to_yaml(msg.number_total, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FizzBuzz & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace fizzbuzz_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use fizzbuzz_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const fizzbuzz_interfaces::msg::FizzBuzz & msg,
  std::ostream & out, size_t indentation = 0)
{
  fizzbuzz_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use fizzbuzz_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const fizzbuzz_interfaces::msg::FizzBuzz & msg)
{
  return fizzbuzz_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<fizzbuzz_interfaces::msg::FizzBuzz>()
{
  return "fizzbuzz_interfaces::msg::FizzBuzz";
}

template<>
inline const char * name<fizzbuzz_interfaces::msg::FizzBuzz>()
{
  return "fizzbuzz_interfaces/msg/FizzBuzz";
}

template<>
struct has_fixed_size<fizzbuzz_interfaces::msg::FizzBuzz>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<fizzbuzz_interfaces::msg::FizzBuzz>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<fizzbuzz_interfaces::msg::FizzBuzz>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // FIZZBUZZ_INTERFACES__MSG__DETAIL__FIZZ_BUZZ__TRAITS_HPP_
