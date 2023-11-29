// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from nmea_msgs:msg/Gpzda.idl
// generated code does not contain a copyright notice
#include "nmea_msgs/msg/detail/gpzda__rosidl_typesupport_fastrtps_cpp.hpp"
#include "nmea_msgs/msg/detail/gpzda__struct.hpp"

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
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs


namespace nmea_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_nmea_msgs
cdr_serialize(
  const nmea_msgs::msg::Gpzda & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: message_id
  cdr << ros_message.message_id;
  // Member: utc_seconds
  cdr << ros_message.utc_seconds;
  // Member: day
  cdr << ros_message.day;
  // Member: month
  cdr << ros_message.month;
  // Member: year
  cdr << ros_message.year;
  // Member: hour_offset_gmt
  cdr << ros_message.hour_offset_gmt;
  // Member: minute_offset_gmt
  cdr << ros_message.minute_offset_gmt;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_nmea_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  nmea_msgs::msg::Gpzda & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: message_id
  cdr >> ros_message.message_id;

  // Member: utc_seconds
  cdr >> ros_message.utc_seconds;

  // Member: day
  cdr >> ros_message.day;

  // Member: month
  cdr >> ros_message.month;

  // Member: year
  cdr >> ros_message.year;

  // Member: hour_offset_gmt
  cdr >> ros_message.hour_offset_gmt;

  // Member: minute_offset_gmt
  cdr >> ros_message.minute_offset_gmt;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_nmea_msgs
get_serialized_size(
  const nmea_msgs::msg::Gpzda & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: message_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.message_id.size() + 1);
  // Member: utc_seconds
  {
    size_t item_size = sizeof(ros_message.utc_seconds);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: day
  {
    size_t item_size = sizeof(ros_message.day);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: month
  {
    size_t item_size = sizeof(ros_message.month);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: year
  {
    size_t item_size = sizeof(ros_message.year);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: hour_offset_gmt
  {
    size_t item_size = sizeof(ros_message.hour_offset_gmt);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: minute_offset_gmt
  {
    size_t item_size = sizeof(ros_message.minute_offset_gmt);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_nmea_msgs
max_serialized_size_Gpzda(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: header
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        full_bounded, current_alignment);
    }
  }

  // Member: message_id
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: utc_seconds
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: day
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: month
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: year
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: hour_offset_gmt
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: minute_offset_gmt
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static bool _Gpzda__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const nmea_msgs::msg::Gpzda *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Gpzda__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<nmea_msgs::msg::Gpzda *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Gpzda__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const nmea_msgs::msg::Gpzda *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Gpzda__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_Gpzda(full_bounded, 0);
}

static message_type_support_callbacks_t _Gpzda__callbacks = {
  "nmea_msgs::msg",
  "Gpzda",
  _Gpzda__cdr_serialize,
  _Gpzda__cdr_deserialize,
  _Gpzda__get_serialized_size,
  _Gpzda__max_serialized_size
};

static rosidl_message_type_support_t _Gpzda__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Gpzda__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace nmea_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_nmea_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<nmea_msgs::msg::Gpzda>()
{
  return &nmea_msgs::msg::typesupport_fastrtps_cpp::_Gpzda__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, nmea_msgs, msg, Gpzda)() {
  return &nmea_msgs::msg::typesupport_fastrtps_cpp::_Gpzda__handle;
}

#ifdef __cplusplus
}
#endif
