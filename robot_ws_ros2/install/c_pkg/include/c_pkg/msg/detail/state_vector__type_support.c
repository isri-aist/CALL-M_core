// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from c_pkg:msg/StateVector.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "c_pkg/msg/detail/state_vector__rosidl_typesupport_introspection_c.h"
#include "c_pkg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "c_pkg/msg/detail/state_vector__functions.h"
#include "c_pkg/msg/detail/state_vector__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void StateVector__rosidl_typesupport_introspection_c__StateVector_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  c_pkg__msg__StateVector__init(message_memory);
}

void StateVector__rosidl_typesupport_introspection_c__StateVector_fini_function(void * message_memory)
{
  c_pkg__msg__StateVector__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember StateVector__rosidl_typesupport_introspection_c__StateVector_message_member_array[3] = {
  {
    "vx",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(c_pkg__msg__StateVector, vx),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "vy",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(c_pkg__msg__StateVector, vy),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "w",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(c_pkg__msg__StateVector, w),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers StateVector__rosidl_typesupport_introspection_c__StateVector_message_members = {
  "c_pkg__msg",  // message namespace
  "StateVector",  // message name
  3,  // number of fields
  sizeof(c_pkg__msg__StateVector),
  StateVector__rosidl_typesupport_introspection_c__StateVector_message_member_array,  // message members
  StateVector__rosidl_typesupport_introspection_c__StateVector_init_function,  // function to initialize message memory (memory has to be allocated)
  StateVector__rosidl_typesupport_introspection_c__StateVector_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t StateVector__rosidl_typesupport_introspection_c__StateVector_message_type_support_handle = {
  0,
  &StateVector__rosidl_typesupport_introspection_c__StateVector_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_c_pkg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, c_pkg, msg, StateVector)() {
  if (!StateVector__rosidl_typesupport_introspection_c__StateVector_message_type_support_handle.typesupport_identifier) {
    StateVector__rosidl_typesupport_introspection_c__StateVector_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &StateVector__rosidl_typesupport_introspection_c__StateVector_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
