// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from c_pkg:msg/StateVector.idl
// generated code does not contain a copyright notice

#ifndef C_PKG__MSG__DETAIL__STATE_VECTOR__STRUCT_H_
#define C_PKG__MSG__DETAIL__STATE_VECTOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/StateVector in the package c_pkg.
typedef struct c_pkg__msg__StateVector
{
  double vx;
  double vy;
  double w;
} c_pkg__msg__StateVector;

// Struct for a sequence of c_pkg__msg__StateVector.
typedef struct c_pkg__msg__StateVector__Sequence
{
  c_pkg__msg__StateVector * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} c_pkg__msg__StateVector__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // C_PKG__MSG__DETAIL__STATE_VECTOR__STRUCT_H_
