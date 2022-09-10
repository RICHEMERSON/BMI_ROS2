// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/PassiveObservation.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__PASSIVE_OBSERVATION__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__PASSIVE_OBSERVATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/passive_observation__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_PassiveObservation_y_shape
{
public:
  explicit Init_PassiveObservation_y_shape(::interfaces::msg::PassiveObservation & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::PassiveObservation y_shape(::interfaces::msg::PassiveObservation::_y_shape_type arg)
  {
    msg_.y_shape = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::PassiveObservation msg_;
};

class Init_PassiveObservation_y_observation
{
public:
  Init_PassiveObservation_y_observation()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PassiveObservation_y_shape y_observation(::interfaces::msg::PassiveObservation::_y_observation_type arg)
  {
    msg_.y_observation = std::move(arg);
    return Init_PassiveObservation_y_shape(msg_);
  }

private:
  ::interfaces::msg::PassiveObservation msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::PassiveObservation>()
{
  return interfaces::msg::builder::Init_PassiveObservation_y_observation();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__PASSIVE_OBSERVATION__BUILDER_HPP_
