// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/DecoderElement.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__DECODER_ELEMENT__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__DECODER_ELEMENT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__DecoderElement __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__DecoderElement __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DecoderElement_
{
  using Type = DecoderElement_<ContainerAllocator>;

  explicit DecoderElement_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit DecoderElement_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _de_type =
    std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>>;
  _de_type de;

  // setters for named parameter idiom
  Type & set__de(
    const std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>> & _arg)
  {
    this->de = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::msg::DecoderElement_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::DecoderElement_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::DecoderElement_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::DecoderElement_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::DecoderElement_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::DecoderElement_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::DecoderElement_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::DecoderElement_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::DecoderElement_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::DecoderElement_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__DecoderElement
    std::shared_ptr<interfaces::msg::DecoderElement_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__DecoderElement
    std::shared_ptr<interfaces::msg::DecoderElement_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DecoderElement_ & other) const
  {
    if (this->de != other.de) {
      return false;
    }
    return true;
  }
  bool operator!=(const DecoderElement_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DecoderElement_

// alias to use template instance with default allocator
using DecoderElement =
  interfaces::msg::DecoderElement_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__DECODER_ELEMENT__STRUCT_HPP_
