// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from interfaces:msg/PassiveObservation.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "interfaces/msg/detail/passive_observation__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void PassiveObservation_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) interfaces::msg::PassiveObservation(_init);
}

void PassiveObservation_fini_function(void * message_memory)
{
  auto typed_message = static_cast<interfaces::msg::PassiveObservation *>(message_memory);
  typed_message->~PassiveObservation();
}

size_t size_function__PassiveObservation__y_observation(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__PassiveObservation__y_observation(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__PassiveObservation__y_observation(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__PassiveObservation__y_observation(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__PassiveObservation__y_observation(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__PassiveObservation__y_observation(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__PassiveObservation__y_observation(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__PassiveObservation__y_observation(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__PassiveObservation__y_shape(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int64_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__PassiveObservation__y_shape(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int64_t> *>(untyped_member);
  return &member[index];
}

void * get_function__PassiveObservation__y_shape(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int64_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__PassiveObservation__y_shape(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int64_t *>(
    get_const_function__PassiveObservation__y_shape(untyped_member, index));
  auto & value = *reinterpret_cast<int64_t *>(untyped_value);
  value = item;
}

void assign_function__PassiveObservation__y_shape(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int64_t *>(
    get_function__PassiveObservation__y_shape(untyped_member, index));
  const auto & value = *reinterpret_cast<const int64_t *>(untyped_value);
  item = value;
}

void resize_function__PassiveObservation__y_shape(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int64_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PassiveObservation_message_member_array[2] = {
  {
    "y_observation",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces::msg::PassiveObservation, y_observation),  // bytes offset in struct
    nullptr,  // default value
    size_function__PassiveObservation__y_observation,  // size() function pointer
    get_const_function__PassiveObservation__y_observation,  // get_const(index) function pointer
    get_function__PassiveObservation__y_observation,  // get(index) function pointer
    fetch_function__PassiveObservation__y_observation,  // fetch(index, &value) function pointer
    assign_function__PassiveObservation__y_observation,  // assign(index, value) function pointer
    resize_function__PassiveObservation__y_observation  // resize(index) function pointer
  },
  {
    "y_shape",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces::msg::PassiveObservation, y_shape),  // bytes offset in struct
    nullptr,  // default value
    size_function__PassiveObservation__y_shape,  // size() function pointer
    get_const_function__PassiveObservation__y_shape,  // get_const(index) function pointer
    get_function__PassiveObservation__y_shape,  // get(index) function pointer
    fetch_function__PassiveObservation__y_shape,  // fetch(index, &value) function pointer
    assign_function__PassiveObservation__y_shape,  // assign(index, value) function pointer
    resize_function__PassiveObservation__y_shape  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PassiveObservation_message_members = {
  "interfaces::msg",  // message namespace
  "PassiveObservation",  // message name
  2,  // number of fields
  sizeof(interfaces::msg::PassiveObservation),
  PassiveObservation_message_member_array,  // message members
  PassiveObservation_init_function,  // function to initialize message memory (memory has to be allocated)
  PassiveObservation_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PassiveObservation_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PassiveObservation_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<interfaces::msg::PassiveObservation>()
{
  return &::interfaces::msg::rosidl_typesupport_introspection_cpp::PassiveObservation_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, interfaces, msg, PassiveObservation)() {
  return &::interfaces::msg::rosidl_typesupport_introspection_cpp::PassiveObservation_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
