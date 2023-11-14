// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from c_pkg:msg/StateVector.idl
// generated code does not contain a copyright notice
#include "c_pkg/msg/detail/state_vector__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "c_pkg/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "c_pkg/msg/detail/state_vector__struct.h"
#include "c_pkg/msg/detail/state_vector__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _StateVector__ros_msg_type = c_pkg__msg__StateVector;

static bool _StateVector__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _StateVector__ros_msg_type * ros_message = static_cast<const _StateVector__ros_msg_type *>(untyped_ros_message);
  // Field name: vx
  {
    cdr << ros_message->vx;
  }

  // Field name: vy
  {
    cdr << ros_message->vy;
  }

  // Field name: w
  {
    cdr << ros_message->w;
  }

  return true;
}

static bool _StateVector__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _StateVector__ros_msg_type * ros_message = static_cast<_StateVector__ros_msg_type *>(untyped_ros_message);
  // Field name: vx
  {
    cdr >> ros_message->vx;
  }

  // Field name: vy
  {
    cdr >> ros_message->vy;
  }

  // Field name: w
  {
    cdr >> ros_message->w;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_c_pkg
size_t get_serialized_size_c_pkg__msg__StateVector(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _StateVector__ros_msg_type * ros_message = static_cast<const _StateVector__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name vx
  {
    size_t item_size = sizeof(ros_message->vx);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name vy
  {
    size_t item_size = sizeof(ros_message->vy);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name w
  {
    size_t item_size = sizeof(ros_message->w);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _StateVector__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_c_pkg__msg__StateVector(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_c_pkg
size_t max_serialized_size_c_pkg__msg__StateVector(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: vx
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: vy
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: w
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _StateVector__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_c_pkg__msg__StateVector(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_StateVector = {
  "c_pkg::msg",
  "StateVector",
  _StateVector__cdr_serialize,
  _StateVector__cdr_deserialize,
  _StateVector__get_serialized_size,
  _StateVector__max_serialized_size
};

static rosidl_message_type_support_t _StateVector__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_StateVector,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, c_pkg, msg, StateVector)() {
  return &_StateVector__type_support;
}

#if defined(__cplusplus)
}
#endif
