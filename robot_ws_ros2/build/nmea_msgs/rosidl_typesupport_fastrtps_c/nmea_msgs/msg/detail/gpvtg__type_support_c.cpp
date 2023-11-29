// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from nmea_msgs:msg/Gpvtg.idl
// generated code does not contain a copyright notice
#include "nmea_msgs/msg/detail/gpvtg__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "nmea_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "nmea_msgs/msg/detail/gpvtg__struct.h"
#include "nmea_msgs/msg/detail/gpvtg__functions.h"
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

#include "rosidl_runtime_c/string.h"  // message_id, mode_indicator, speed_k_unit, speed_n_unit, track_m_ref, track_t_ref
#include "rosidl_runtime_c/string_functions.h"  // message_id, mode_indicator, speed_k_unit, speed_n_unit, track_m_ref, track_t_ref
#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_nmea_msgs
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_nmea_msgs
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_nmea_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _Gpvtg__ros_msg_type = nmea_msgs__msg__Gpvtg;

static bool _Gpvtg__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Gpvtg__ros_msg_type * ros_message = static_cast<const _Gpvtg__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: message_id
  {
    const rosidl_runtime_c__String * str = &ros_message->message_id;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: track_t
  {
    cdr << ros_message->track_t;
  }

  // Field name: track_t_ref
  {
    const rosidl_runtime_c__String * str = &ros_message->track_t_ref;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: track_m
  {
    cdr << ros_message->track_m;
  }

  // Field name: track_m_ref
  {
    const rosidl_runtime_c__String * str = &ros_message->track_m_ref;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: speed_n
  {
    cdr << ros_message->speed_n;
  }

  // Field name: speed_n_unit
  {
    const rosidl_runtime_c__String * str = &ros_message->speed_n_unit;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: speed_k
  {
    cdr << ros_message->speed_k;
  }

  // Field name: speed_k_unit
  {
    const rosidl_runtime_c__String * str = &ros_message->speed_k_unit;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: mode_indicator
  {
    const rosidl_runtime_c__String * str = &ros_message->mode_indicator;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _Gpvtg__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Gpvtg__ros_msg_type * ros_message = static_cast<_Gpvtg__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: message_id
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->message_id.data) {
      rosidl_runtime_c__String__init(&ros_message->message_id);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->message_id,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'message_id'\n");
      return false;
    }
  }

  // Field name: track_t
  {
    cdr >> ros_message->track_t;
  }

  // Field name: track_t_ref
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->track_t_ref.data) {
      rosidl_runtime_c__String__init(&ros_message->track_t_ref);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->track_t_ref,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'track_t_ref'\n");
      return false;
    }
  }

  // Field name: track_m
  {
    cdr >> ros_message->track_m;
  }

  // Field name: track_m_ref
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->track_m_ref.data) {
      rosidl_runtime_c__String__init(&ros_message->track_m_ref);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->track_m_ref,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'track_m_ref'\n");
      return false;
    }
  }

  // Field name: speed_n
  {
    cdr >> ros_message->speed_n;
  }

  // Field name: speed_n_unit
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->speed_n_unit.data) {
      rosidl_runtime_c__String__init(&ros_message->speed_n_unit);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->speed_n_unit,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'speed_n_unit'\n");
      return false;
    }
  }

  // Field name: speed_k
  {
    cdr >> ros_message->speed_k;
  }

  // Field name: speed_k_unit
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->speed_k_unit.data) {
      rosidl_runtime_c__String__init(&ros_message->speed_k_unit);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->speed_k_unit,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'speed_k_unit'\n");
      return false;
    }
  }

  // Field name: mode_indicator
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->mode_indicator.data) {
      rosidl_runtime_c__String__init(&ros_message->mode_indicator);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->mode_indicator,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'mode_indicator'\n");
      return false;
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_nmea_msgs
size_t get_serialized_size_nmea_msgs__msg__Gpvtg(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Gpvtg__ros_msg_type * ros_message = static_cast<const _Gpvtg__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name message_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->message_id.size + 1);
  // field.name track_t
  {
    size_t item_size = sizeof(ros_message->track_t);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name track_t_ref
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->track_t_ref.size + 1);
  // field.name track_m
  {
    size_t item_size = sizeof(ros_message->track_m);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name track_m_ref
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->track_m_ref.size + 1);
  // field.name speed_n
  {
    size_t item_size = sizeof(ros_message->speed_n);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name speed_n_unit
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->speed_n_unit.size + 1);
  // field.name speed_k
  {
    size_t item_size = sizeof(ros_message->speed_k);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name speed_k_unit
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->speed_k_unit.size + 1);
  // field.name mode_indicator
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->mode_indicator.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _Gpvtg__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_nmea_msgs__msg__Gpvtg(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_nmea_msgs
size_t max_serialized_size_nmea_msgs__msg__Gpvtg(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: header
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_std_msgs__msg__Header(
        full_bounded, current_alignment);
    }
  }
  // member: message_id
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: track_t
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: track_t_ref
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: track_m
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: track_m_ref
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: speed_n
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: speed_n_unit
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: speed_k
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: speed_k_unit
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: mode_indicator
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _Gpvtg__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_nmea_msgs__msg__Gpvtg(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_Gpvtg = {
  "nmea_msgs::msg",
  "Gpvtg",
  _Gpvtg__cdr_serialize,
  _Gpvtg__cdr_deserialize,
  _Gpvtg__get_serialized_size,
  _Gpvtg__max_serialized_size
};

static rosidl_message_type_support_t _Gpvtg__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Gpvtg,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, nmea_msgs, msg, Gpvtg)() {
  return &_Gpvtg__type_support;
}

#if defined(__cplusplus)
}
#endif
