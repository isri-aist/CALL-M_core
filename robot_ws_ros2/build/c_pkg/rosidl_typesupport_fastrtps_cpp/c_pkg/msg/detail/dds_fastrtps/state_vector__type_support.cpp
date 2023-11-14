// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from c_pkg:msg/StateVector.idl
// generated code does not contain a copyright notice
#include "c_pkg/msg/detail/state_vector__rosidl_typesupport_fastrtps_cpp.hpp"
#include "c_pkg/msg/detail/state_vector__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace c_pkg
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_c_pkg
cdr_serialize(
  const c_pkg::msg::StateVector & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: vx
  cdr << ros_message.vx;
  // Member: vy
  cdr << ros_message.vy;
  // Member: w
  cdr << ros_message.w;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_c_pkg
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  c_pkg::msg::StateVector & ros_message)
{
  // Member: vx
  cdr >> ros_message.vx;

  // Member: vy
  cdr >> ros_message.vy;

  // Member: w
  cdr >> ros_message.w;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_c_pkg
get_serialized_size(
  const c_pkg::msg::StateVector & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: vx
  {
    size_t item_size = sizeof(ros_message.vx);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: vy
  {
    size_t item_size = sizeof(ros_message.vy);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: w
  {
    size_t item_size = sizeof(ros_message.w);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_c_pkg
max_serialized_size_StateVector(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: vx
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: vy
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: w
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  return current_alignment - initial_alignment;
}

static bool _StateVector__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const c_pkg::msg::StateVector *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _StateVector__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<c_pkg::msg::StateVector *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _StateVector__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const c_pkg::msg::StateVector *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _StateVector__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_StateVector(full_bounded, 0);
}

static message_type_support_callbacks_t _StateVector__callbacks = {
  "c_pkg::msg",
  "StateVector",
  _StateVector__cdr_serialize,
  _StateVector__cdr_deserialize,
  _StateVector__get_serialized_size,
  _StateVector__max_serialized_size
};

static rosidl_message_type_support_t _StateVector__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_StateVector__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace c_pkg

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_c_pkg
const rosidl_message_type_support_t *
get_message_type_support_handle<c_pkg::msg::StateVector>()
{
  return &c_pkg::msg::typesupport_fastrtps_cpp::_StateVector__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, c_pkg, msg, StateVector)() {
  return &c_pkg::msg::typesupport_fastrtps_cpp::_StateVector__handle;
}

#ifdef __cplusplus
}
#endif
