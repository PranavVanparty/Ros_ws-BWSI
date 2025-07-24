// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from fizzbuzz_interfaces:msg/FizzBuzz.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "fizzbuzz_interfaces/msg/fizz_buzz.hpp"


#ifndef FIZZBUZZ_INTERFACES__MSG__DETAIL__FIZZ_BUZZ__BUILDER_HPP_
#define FIZZBUZZ_INTERFACES__MSG__DETAIL__FIZZ_BUZZ__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "fizzbuzz_interfaces/msg/detail/fizz_buzz__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace fizzbuzz_interfaces
{

namespace msg
{

namespace builder
{

class Init_FizzBuzz_number_total
{
public:
  explicit Init_FizzBuzz_number_total(::fizzbuzz_interfaces::msg::FizzBuzz & msg)
  : msg_(msg)
  {}
  ::fizzbuzz_interfaces::msg::FizzBuzz number_total(::fizzbuzz_interfaces::msg::FizzBuzz::_number_total_type arg)
  {
    msg_.number_total = std::move(arg);
    return std::move(msg_);
  }

private:
  ::fizzbuzz_interfaces::msg::FizzBuzz msg_;
};

class Init_FizzBuzz_fizzbuzz_ratio
{
public:
  explicit Init_FizzBuzz_fizzbuzz_ratio(::fizzbuzz_interfaces::msg::FizzBuzz & msg)
  : msg_(msg)
  {}
  Init_FizzBuzz_number_total fizzbuzz_ratio(::fizzbuzz_interfaces::msg::FizzBuzz::_fizzbuzz_ratio_type arg)
  {
    msg_.fizzbuzz_ratio = std::move(arg);
    return Init_FizzBuzz_number_total(msg_);
  }

private:
  ::fizzbuzz_interfaces::msg::FizzBuzz msg_;
};

class Init_FizzBuzz_buzz_ratio
{
public:
  explicit Init_FizzBuzz_buzz_ratio(::fizzbuzz_interfaces::msg::FizzBuzz & msg)
  : msg_(msg)
  {}
  Init_FizzBuzz_fizzbuzz_ratio buzz_ratio(::fizzbuzz_interfaces::msg::FizzBuzz::_buzz_ratio_type arg)
  {
    msg_.buzz_ratio = std::move(arg);
    return Init_FizzBuzz_fizzbuzz_ratio(msg_);
  }

private:
  ::fizzbuzz_interfaces::msg::FizzBuzz msg_;
};

class Init_FizzBuzz_fizz_ratio
{
public:
  explicit Init_FizzBuzz_fizz_ratio(::fizzbuzz_interfaces::msg::FizzBuzz & msg)
  : msg_(msg)
  {}
  Init_FizzBuzz_buzz_ratio fizz_ratio(::fizzbuzz_interfaces::msg::FizzBuzz::_fizz_ratio_type arg)
  {
    msg_.fizz_ratio = std::move(arg);
    return Init_FizzBuzz_buzz_ratio(msg_);
  }

private:
  ::fizzbuzz_interfaces::msg::FizzBuzz msg_;
};

class Init_FizzBuzz_fizzbuzz
{
public:
  Init_FizzBuzz_fizzbuzz()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FizzBuzz_fizz_ratio fizzbuzz(::fizzbuzz_interfaces::msg::FizzBuzz::_fizzbuzz_type arg)
  {
    msg_.fizzbuzz = std::move(arg);
    return Init_FizzBuzz_fizz_ratio(msg_);
  }

private:
  ::fizzbuzz_interfaces::msg::FizzBuzz msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::fizzbuzz_interfaces::msg::FizzBuzz>()
{
  return fizzbuzz_interfaces::msg::builder::Init_FizzBuzz_fizzbuzz();
}

}  // namespace fizzbuzz_interfaces

#endif  // FIZZBUZZ_INTERFACES__MSG__DETAIL__FIZZ_BUZZ__BUILDER_HPP_
