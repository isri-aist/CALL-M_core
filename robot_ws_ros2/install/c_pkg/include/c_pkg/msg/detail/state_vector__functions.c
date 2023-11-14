// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from c_pkg:msg/StateVector.idl
// generated code does not contain a copyright notice
#include "c_pkg/msg/detail/state_vector__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
c_pkg__msg__StateVector__init(c_pkg__msg__StateVector * msg)
{
  if (!msg) {
    return false;
  }
  // vx
  // vy
  // w
  return true;
}

void
c_pkg__msg__StateVector__fini(c_pkg__msg__StateVector * msg)
{
  if (!msg) {
    return;
  }
  // vx
  // vy
  // w
}

bool
c_pkg__msg__StateVector__are_equal(const c_pkg__msg__StateVector * lhs, const c_pkg__msg__StateVector * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // vx
  if (lhs->vx != rhs->vx) {
    return false;
  }
  // vy
  if (lhs->vy != rhs->vy) {
    return false;
  }
  // w
  if (lhs->w != rhs->w) {
    return false;
  }
  return true;
}

bool
c_pkg__msg__StateVector__copy(
  const c_pkg__msg__StateVector * input,
  c_pkg__msg__StateVector * output)
{
  if (!input || !output) {
    return false;
  }
  // vx
  output->vx = input->vx;
  // vy
  output->vy = input->vy;
  // w
  output->w = input->w;
  return true;
}

c_pkg__msg__StateVector *
c_pkg__msg__StateVector__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  c_pkg__msg__StateVector * msg = (c_pkg__msg__StateVector *)allocator.allocate(sizeof(c_pkg__msg__StateVector), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(c_pkg__msg__StateVector));
  bool success = c_pkg__msg__StateVector__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
c_pkg__msg__StateVector__destroy(c_pkg__msg__StateVector * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    c_pkg__msg__StateVector__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
c_pkg__msg__StateVector__Sequence__init(c_pkg__msg__StateVector__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  c_pkg__msg__StateVector * data = NULL;

  if (size) {
    data = (c_pkg__msg__StateVector *)allocator.zero_allocate(size, sizeof(c_pkg__msg__StateVector), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = c_pkg__msg__StateVector__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        c_pkg__msg__StateVector__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
c_pkg__msg__StateVector__Sequence__fini(c_pkg__msg__StateVector__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      c_pkg__msg__StateVector__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

c_pkg__msg__StateVector__Sequence *
c_pkg__msg__StateVector__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  c_pkg__msg__StateVector__Sequence * array = (c_pkg__msg__StateVector__Sequence *)allocator.allocate(sizeof(c_pkg__msg__StateVector__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = c_pkg__msg__StateVector__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
c_pkg__msg__StateVector__Sequence__destroy(c_pkg__msg__StateVector__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    c_pkg__msg__StateVector__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
c_pkg__msg__StateVector__Sequence__are_equal(const c_pkg__msg__StateVector__Sequence * lhs, const c_pkg__msg__StateVector__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!c_pkg__msg__StateVector__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
c_pkg__msg__StateVector__Sequence__copy(
  const c_pkg__msg__StateVector__Sequence * input,
  c_pkg__msg__StateVector__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(c_pkg__msg__StateVector);
    c_pkg__msg__StateVector * data =
      (c_pkg__msg__StateVector *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!c_pkg__msg__StateVector__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          c_pkg__msg__StateVector__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!c_pkg__msg__StateVector__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
