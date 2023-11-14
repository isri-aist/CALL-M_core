// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from c_pkg:msg/StateVector.idl
// generated code does not contain a copyright notice

#ifndef C_PKG__MSG__DETAIL__STATE_VECTOR__BUILDER_HPP_
#define C_PKG__MSG__DETAIL__STATE_VECTOR__BUILDER_HPP_

#include "c_pkg/msg/detail/state_vector__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace c_pkg
{

namespace msg
{

namespace builder
{

class Init_StateVector_w
{
public:
  explicit Init_StateVector_w(::c_pkg::msg::StateVector & msg)
  : msg_(msg)
  {}
  ::c_pkg::msg::StateVector w(::c_pkg::msg::StateVector::_w_type arg)
  {
    msg_.w = std::move(arg);
    return std::move(msg_);
  }

private:
  ::c_pkg::msg::StateVector msg_;
};

class Init_StateVector_vy
{
public:
  explicit Init_StateVector_vy(::c_pkg::msg::StateVector & msg)
  : msg_(msg)
  {}
  Init_StateVector_w vy(::c_pkg::msg::StateVector::_vy_type arg)
  {
    msg_.vy = std::move(arg);
    return Init_StateVector_w(msg_);
  }

private:
  ::c_pkg::msg::StateVector msg_;
};

class Init_StateVector_vx
{
public:
  Init_StateVector_vx()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StateVector_vy vx(::c_pkg::msg::StateVector::_vx_type arg)
  {
    msg_.vx = std::move(arg);
    return Init_StateVector_vy(msg_);
  }

private:
  ::c_pkg::msg::StateVector msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::c_pkg::msg::StateVector>()
{
  return c_pkg::msg::builder::Init_StateVector_vx();
}

}  // namespace c_pkg

#endif  // C_PKG__MSG__DETAIL__STATE_VECTOR__BUILDER_HPP_
