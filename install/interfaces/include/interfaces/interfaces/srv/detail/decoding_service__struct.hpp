// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:srv/DecodingService.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__DECODING_SERVICE__STRUCT_HPP_
#define INTERFACES__SRV__DETAIL__DECODING_SERVICE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__srv__DecodingService_Request __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__srv__DecodingService_Request __declspec(deprecated)
#endif

namespace interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct DecodingService_Request_
{
  using Type = DecodingService_Request_<ContainerAllocator>;

  explicit DecodingService_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->req = false;
    }
  }

  explicit DecodingService_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->req = false;
    }
  }

  // field types and members
  using _req_type =
    bool;
  _req_type req;

  // setters for named parameter idiom
  Type & set__req(
    const bool & _arg)
  {
    this->req = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::srv::DecodingService_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::srv::DecodingService_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::srv::DecodingService_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::srv::DecodingService_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::srv::DecodingService_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::srv::DecodingService_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::srv::DecodingService_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::srv::DecodingService_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::srv::DecodingService_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::srv::DecodingService_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__srv__DecodingService_Request
    std::shared_ptr<interfaces::srv::DecodingService_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__srv__DecodingService_Request
    std::shared_ptr<interfaces::srv::DecodingService_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DecodingService_Request_ & other) const
  {
    if (this->req != other.req) {
      return false;
    }
    return true;
  }
  bool operator!=(const DecodingService_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DecodingService_Request_

// alias to use template instance with default allocator
using DecodingService_Request =
  interfaces::srv::DecodingService_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interfaces


#ifndef _WIN32
# define DEPRECATED__interfaces__srv__DecodingService_Response __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__srv__DecodingService_Response __declspec(deprecated)
#endif

namespace interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct DecodingService_Response_
{
  using Type = DecodingService_Response_<ContainerAllocator>;

  explicit DecodingService_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit DecodingService_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _res_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _res_type res;

  // setters for named parameter idiom
  Type & set__res(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->res = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::srv::DecodingService_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::srv::DecodingService_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::srv::DecodingService_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::srv::DecodingService_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::srv::DecodingService_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::srv::DecodingService_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::srv::DecodingService_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::srv::DecodingService_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::srv::DecodingService_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::srv::DecodingService_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__srv__DecodingService_Response
    std::shared_ptr<interfaces::srv::DecodingService_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__srv__DecodingService_Response
    std::shared_ptr<interfaces::srv::DecodingService_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DecodingService_Response_ & other) const
  {
    if (this->res != other.res) {
      return false;
    }
    return true;
  }
  bool operator!=(const DecodingService_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DecodingService_Response_

// alias to use template instance with default allocator
using DecodingService_Response =
  interfaces::srv::DecodingService_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interfaces

namespace interfaces
{

namespace srv
{

struct DecodingService
{
  using Request = interfaces::srv::DecodingService_Request;
  using Response = interfaces::srv::DecodingService_Response;
};

}  // namespace srv

}  // namespace interfaces

#endif  // INTERFACES__SRV__DETAIL__DECODING_SERVICE__STRUCT_HPP_
