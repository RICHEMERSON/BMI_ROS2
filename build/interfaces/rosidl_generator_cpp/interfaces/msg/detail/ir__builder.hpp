// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/Ir.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__IR__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__IR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/ir__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_Ir_ir
{
public:
  Init_Ir_ir()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interfaces::msg::Ir ir(::interfaces::msg::Ir::_ir_type arg)
  {
    msg_.ir = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::Ir msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::Ir>()
{
  return interfaces::msg::builder::Init_Ir_ir();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__IR__BUILDER_HPP_
