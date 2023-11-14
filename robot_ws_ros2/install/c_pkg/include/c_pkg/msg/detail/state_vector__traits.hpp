// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from c_pkg:msg/StateVector.idl
// generated code does not contain a copyright notice

#ifndef C_PKG__MSG__DETAIL__STATE_VECTOR__TRAITS_HPP_
#define C_PKG__MSG__DETAIL__STATE_VECTOR__TRAITS_HPP_

#include "c_pkg/msg/detail/state_vector__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<c_pkg::msg::StateVector>()
{
  return "c_pkg::msg::StateVector";
}

template<>
inline const char * name<c_pkg::msg::StateVector>()
{
  return "c_pkg/msg/StateVector";
}

template<>
struct has_fixed_size<c_pkg::msg::StateVector>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<c_pkg::msg::StateVector>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<c_pkg::msg::StateVector>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // C_PKG__MSG__DETAIL__STATE_VECTOR__TRAITS_HPP_
