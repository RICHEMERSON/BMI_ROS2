// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/PassiveObservation.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__PASSIVE_OBSERVATION__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__PASSIVE_OBSERVATION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__PassiveObservation __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__PassiveObservation __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PassiveObservation_
{
  using Type = PassiveObservation_<ContainerAllocator>;

  explicit PassiveObservation_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit PassiveObservation_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _y_observation_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _y_observation_type y_observation;
  using _y_shape_type =
    std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>>;
  _y_shape_type y_shape;

  // setters for named parameter idiom
  Type & set__y_observation(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->y_observation = _arg;
    return *this;
  }
  Type & set__y_shape(
    const std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>> & _arg)
  {
    this->y_shape = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::msg::PassiveObservation_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::PassiveObservation_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::PassiveObservation_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::PassiveObservation_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::PassiveObservation_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::PassiveObservation_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::PassiveObservation_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::PassiveObservation_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::PassiveObservation_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::PassiveObservation_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__PassiveObservation
    std::shared_ptr<interfaces::msg::PassiveObservation_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__PassiveObservation
    std::shared_ptr<interfaces::msg::PassiveObservation_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PassiveObservation_ & other) const
  {
    if (this->y_observation != other.y_observation) {
      return false;
    }
    if (this->y_shape != other.y_shape) {
      return false;
    }
    return true;
  }
  bool operator!=(const PassiveObservation_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PassiveObservation_

// alias to use template instance with default allocator
using PassiveObservation =
  interfaces::msg::PassiveObservation_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__PASSIVE_OBSERVATION__STRUCT_HPP_
