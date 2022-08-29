// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/Sample.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__SAMPLE__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__SAMPLE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__Sample __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__Sample __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Sample_
{
  using Type = Sample_<ContainerAllocator>;

  explicit Sample_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Sample_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _x_state_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _x_state_type x_state;
  using _y_observation_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _y_observation_type y_observation;

  // setters for named parameter idiom
  Type & set__x_state(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->x_state = _arg;
    return *this;
  }
  Type & set__y_observation(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->y_observation = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::msg::Sample_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::Sample_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::Sample_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::Sample_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::Sample_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::Sample_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::Sample_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::Sample_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::Sample_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::Sample_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__Sample
    std::shared_ptr<interfaces::msg::Sample_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__Sample
    std::shared_ptr<interfaces::msg::Sample_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Sample_ & other) const
  {
    if (this->x_state != other.x_state) {
      return false;
    }
    if (this->y_observation != other.y_observation) {
      return false;
    }
    return true;
  }
  bool operator!=(const Sample_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Sample_

// alias to use template instance with default allocator
using Sample =
  interfaces::msg::Sample_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__SAMPLE__STRUCT_HPP_
