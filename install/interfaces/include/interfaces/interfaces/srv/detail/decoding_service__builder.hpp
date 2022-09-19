// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:srv/DecodingService.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__DECODING_SERVICE__BUILDER_HPP_
#define INTERFACES__SRV__DETAIL__DECODING_SERVICE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/srv/detail/decoding_service__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace srv
{

namespace builder
{

class Init_DecodingService_Request_req
{
public:
  Init_DecodingService_Request_req()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interfaces::srv::DecodingService_Request req(::interfaces::srv::DecodingService_Request::_req_type arg)
  {
    msg_.req = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::srv::DecodingService_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::srv::DecodingService_Request>()
{
  return interfaces::srv::builder::Init_DecodingService_Request_req();
}

}  // namespace interfaces


namespace interfaces
{

namespace srv
{

namespace builder
{

class Init_DecodingService_Response_res
{
public:
  Init_DecodingService_Response_res()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interfaces::srv::DecodingService_Response res(::interfaces::srv::DecodingService_Response::_res_type arg)
  {
    msg_.res = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::srv::DecodingService_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::srv::DecodingService_Response>()
{
  return interfaces::srv::builder::Init_DecodingService_Response_res();
}

}  // namespace interfaces

#endif  // INTERFACES__SRV__DETAIL__DECODING_SERVICE__BUILDER_HPP_
