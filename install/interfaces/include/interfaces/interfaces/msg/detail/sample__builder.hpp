// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/Sample.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__SAMPLE__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__SAMPLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/sample__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_Sample_y_observation
{
public:
  explicit Init_Sample_y_observation(::interfaces::msg::Sample & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::Sample y_observation(::interfaces::msg::Sample::_y_observation_type arg)
  {
    msg_.y_observation = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::Sample msg_;
};

class Init_Sample_x_state
{
public:
  Init_Sample_x_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Sample_y_observation x_state(::interfaces::msg::Sample::_x_state_type arg)
  {
    msg_.x_state = std::move(arg);
    return Init_Sample_y_observation(msg_);
  }

private:
  ::interfaces::msg::Sample msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::Sample>()
{
  return interfaces::msg::builder::Init_Sample_x_state();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__SAMPLE__BUILDER_HPP_
