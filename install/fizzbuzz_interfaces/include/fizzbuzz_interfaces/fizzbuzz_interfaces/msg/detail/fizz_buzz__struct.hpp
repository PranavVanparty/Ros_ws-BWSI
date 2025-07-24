// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from fizzbuzz_interfaces:msg/FizzBuzz.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "fizzbuzz_interfaces/msg/fizz_buzz.hpp"


#ifndef FIZZBUZZ_INTERFACES__MSG__DETAIL__FIZZ_BUZZ__STRUCT_HPP_
#define FIZZBUZZ_INTERFACES__MSG__DETAIL__FIZZ_BUZZ__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__fizzbuzz_interfaces__msg__FizzBuzz __attribute__((deprecated))
#else
# define DEPRECATED__fizzbuzz_interfaces__msg__FizzBuzz __declspec(deprecated)
#endif

namespace fizzbuzz_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FizzBuzz_
{
  using Type = FizzBuzz_<ContainerAllocator>;

  explicit FizzBuzz_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fizzbuzz = "";
      this->fizz_ratio = 0.0f;
      this->buzz_ratio = 0.0f;
      this->fizzbuzz_ratio = 0.0f;
      this->number_total = 0l;
    }
  }

  explicit FizzBuzz_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : fizzbuzz(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fizzbuzz = "";
      this->fizz_ratio = 0.0f;
      this->buzz_ratio = 0.0f;
      this->fizzbuzz_ratio = 0.0f;
      this->number_total = 0l;
    }
  }

  // field types and members
  using _fizzbuzz_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _fizzbuzz_type fizzbuzz;
  using _fizz_ratio_type =
    float;
  _fizz_ratio_type fizz_ratio;
  using _buzz_ratio_type =
    float;
  _buzz_ratio_type buzz_ratio;
  using _fizzbuzz_ratio_type =
    float;
  _fizzbuzz_ratio_type fizzbuzz_ratio;
  using _number_total_type =
    int32_t;
  _number_total_type number_total;

  // setters for named parameter idiom
  Type & set__fizzbuzz(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->fizzbuzz = _arg;
    return *this;
  }
  Type & set__fizz_ratio(
    const float & _arg)
  {
    this->fizz_ratio = _arg;
    return *this;
  }
  Type & set__buzz_ratio(
    const float & _arg)
  {
    this->buzz_ratio = _arg;
    return *this;
  }
  Type & set__fizzbuzz_ratio(
    const float & _arg)
  {
    this->fizzbuzz_ratio = _arg;
    return *this;
  }
  Type & set__number_total(
    const int32_t & _arg)
  {
    this->number_total = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    fizzbuzz_interfaces::msg::FizzBuzz_<ContainerAllocator> *;
  using ConstRawPtr =
    const fizzbuzz_interfaces::msg::FizzBuzz_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<fizzbuzz_interfaces::msg::FizzBuzz_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<fizzbuzz_interfaces::msg::FizzBuzz_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      fizzbuzz_interfaces::msg::FizzBuzz_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<fizzbuzz_interfaces::msg::FizzBuzz_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      fizzbuzz_interfaces::msg::FizzBuzz_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<fizzbuzz_interfaces::msg::FizzBuzz_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<fizzbuzz_interfaces::msg::FizzBuzz_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<fizzbuzz_interfaces::msg::FizzBuzz_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__fizzbuzz_interfaces__msg__FizzBuzz
    std::shared_ptr<fizzbuzz_interfaces::msg::FizzBuzz_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__fizzbuzz_interfaces__msg__FizzBuzz
    std::shared_ptr<fizzbuzz_interfaces::msg::FizzBuzz_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FizzBuzz_ & other) const
  {
    if (this->fizzbuzz != other.fizzbuzz) {
      return false;
    }
    if (this->fizz_ratio != other.fizz_ratio) {
      return false;
    }
    if (this->buzz_ratio != other.buzz_ratio) {
      return false;
    }
    if (this->fizzbuzz_ratio != other.fizzbuzz_ratio) {
      return false;
    }
    if (this->number_total != other.number_total) {
      return false;
    }
    return true;
  }
  bool operator!=(const FizzBuzz_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FizzBuzz_

// alias to use template instance with default allocator
using FizzBuzz =
  fizzbuzz_interfaces::msg::FizzBuzz_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace fizzbuzz_interfaces

#endif  // FIZZBUZZ_INTERFACES__MSG__DETAIL__FIZZ_BUZZ__STRUCT_HPP_
