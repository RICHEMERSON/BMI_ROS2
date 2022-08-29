// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/DecoderElement.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__DECODER_ELEMENT__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__DECODER_ELEMENT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/decoder_element__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_DecoderElement_de
{
public:
  Init_DecoderElement_de()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interfaces::msg::DecoderElement de(::interfaces::msg::DecoderElement::_de_type arg)
  {
    msg_.de = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::DecoderElement msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::DecoderElement>()
{
  return interfaces::msg::builder::Init_DecoderElement_de();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__DECODER_ELEMENT__BUILDER_HPP_
