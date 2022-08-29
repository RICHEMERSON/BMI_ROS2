// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:srv/Decoder.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__DECODER__BUILDER_HPP_
#define INTERFACES__SRV__DETAIL__DECODER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/srv/detail/decoder__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace srv
{

namespace builder
{

class Init_Decoder_Request_req
{
public:
  Init_Decoder_Request_req()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interfaces::srv::Decoder_Request req(::interfaces::srv::Decoder_Request::_req_type arg)
  {
    msg_.req = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::srv::Decoder_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::srv::Decoder_Request>()
{
  return interfaces::srv::builder::Init_Decoder_Request_req();
}

}  // namespace interfaces


namespace interfaces
{

namespace srv
{

namespace builder
{

class Init_Decoder_Response_res
{
public:
  Init_Decoder_Response_res()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interfaces::srv::Decoder_Response res(::interfaces::srv::Decoder_Response::_res_type arg)
  {
    msg_.res = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::srv::Decoder_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::srv::Decoder_Response>()
{
  return interfaces::srv::builder::Init_Decoder_Response_res();
}

}  // namespace interfaces

#endif  // INTERFACES__SRV__DETAIL__DECODER__BUILDER_HPP_
