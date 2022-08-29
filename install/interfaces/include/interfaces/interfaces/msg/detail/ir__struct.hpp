// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/Ir.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__IR__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__IR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__Ir __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__Ir __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Ir_
{
  using Type = Ir_<ContainerAllocator>;

  explicit Ir_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Ir_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _ir_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _ir_type ir;

  // setters for named parameter idiom
  Type & set__ir(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->ir = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::msg::Ir_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::Ir_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::Ir_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::Ir_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::Ir_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::Ir_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::Ir_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::Ir_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::Ir_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::Ir_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__Ir
    std::shared_ptr<interfaces::msg::Ir_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__Ir
    std::shared_ptr<interfaces::msg::Ir_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Ir_ & other) const
  {
    if (this->ir != other.ir) {
      return false;
    }
    return true;
  }
  bool operator!=(const Ir_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Ir_

// alias to use template instance with default allocator
using Ir =
  interfaces::msg::Ir_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__IR__STRUCT_HPP_
