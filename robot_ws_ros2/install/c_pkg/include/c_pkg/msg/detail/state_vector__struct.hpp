// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from c_pkg:msg/StateVector.idl
// generated code does not contain a copyright notice

#ifndef C_PKG__MSG__DETAIL__STATE_VECTOR__STRUCT_HPP_
#define C_PKG__MSG__DETAIL__STATE_VECTOR__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__c_pkg__msg__StateVector __attribute__((deprecated))
#else
# define DEPRECATED__c_pkg__msg__StateVector __declspec(deprecated)
#endif

namespace c_pkg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct StateVector_
{
  using Type = StateVector_<ContainerAllocator>;

  explicit StateVector_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vx = 0.0;
      this->vy = 0.0;
      this->w = 0.0;
    }
  }

  explicit StateVector_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vx = 0.0;
      this->vy = 0.0;
      this->w = 0.0;
    }
  }

  // field types and members
  using _vx_type =
    double;
  _vx_type vx;
  using _vy_type =
    double;
  _vy_type vy;
  using _w_type =
    double;
  _w_type w;

  // setters for named parameter idiom
  Type & set__vx(
    const double & _arg)
  {
    this->vx = _arg;
    return *this;
  }
  Type & set__vy(
    const double & _arg)
  {
    this->vy = _arg;
    return *this;
  }
  Type & set__w(
    const double & _arg)
  {
    this->w = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    c_pkg::msg::StateVector_<ContainerAllocator> *;
  using ConstRawPtr =
    const c_pkg::msg::StateVector_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<c_pkg::msg::StateVector_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<c_pkg::msg::StateVector_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      c_pkg::msg::StateVector_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<c_pkg::msg::StateVector_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      c_pkg::msg::StateVector_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<c_pkg::msg::StateVector_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<c_pkg::msg::StateVector_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<c_pkg::msg::StateVector_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__c_pkg__msg__StateVector
    std::shared_ptr<c_pkg::msg::StateVector_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__c_pkg__msg__StateVector
    std::shared_ptr<c_pkg::msg::StateVector_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StateVector_ & other) const
  {
    if (this->vx != other.vx) {
      return false;
    }
    if (this->vy != other.vy) {
      return false;
    }
    if (this->w != other.w) {
      return false;
    }
    return true;
  }
  bool operator!=(const StateVector_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StateVector_

// alias to use template instance with default allocator
using StateVector =
  c_pkg::msg::StateVector_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace c_pkg

#endif  // C_PKG__MSG__DETAIL__STATE_VECTOR__STRUCT_HPP_
