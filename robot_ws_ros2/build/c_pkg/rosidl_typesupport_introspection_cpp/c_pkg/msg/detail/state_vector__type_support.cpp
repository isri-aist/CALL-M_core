// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from c_pkg:msg/StateVector.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "c_pkg/msg/detail/state_vector__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace c_pkg
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void StateVector_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) c_pkg::msg::StateVector(_init);
}

void StateVector_fini_function(void * message_memory)
{
  auto typed_message = static_cast<c_pkg::msg::StateVector *>(message_memory);
  typed_message->~StateVector();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember StateVector_message_member_array[3] = {
  {
    "vx",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(c_pkg::msg::StateVector, vx),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "vy",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(c_pkg::msg::StateVector, vy),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "w",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(c_pkg::msg::StateVector, w),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers StateVector_message_members = {
  "c_pkg::msg",  // message namespace
  "StateVector",  // message name
  3,  // number of fields
  sizeof(c_pkg::msg::StateVector),
  StateVector_message_member_array,  // message members
  StateVector_init_function,  // function to initialize message memory (memory has to be allocated)
  StateVector_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t StateVector_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &StateVector_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace c_pkg


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<c_pkg::msg::StateVector>()
{
  return &::c_pkg::msg::rosidl_typesupport_introspection_cpp::StateVector_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, c_pkg, msg, StateVector)() {
  return &::c_pkg::msg::rosidl_typesupport_introspection_cpp::StateVector_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
