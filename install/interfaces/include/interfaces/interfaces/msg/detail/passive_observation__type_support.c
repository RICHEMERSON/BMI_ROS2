// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from interfaces:msg/PassiveObservation.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "interfaces/msg/detail/passive_observation__rosidl_typesupport_introspection_c.h"
#include "interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "interfaces/msg/detail/passive_observation__functions.h"
#include "interfaces/msg/detail/passive_observation__struct.h"


// Include directives for member types
// Member `y_observation`
// Member `y_shape`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__PassiveObservation_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  interfaces__msg__PassiveObservation__init(message_memory);
}

void interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__PassiveObservation_fini_function(void * message_memory)
{
  interfaces__msg__PassiveObservation__fini(message_memory);
}

size_t interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__size_function__PassiveObservation__y_observation(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__get_const_function__PassiveObservation__y_observation(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__get_function__PassiveObservation__y_observation(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__fetch_function__PassiveObservation__y_observation(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__get_const_function__PassiveObservation__y_observation(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__assign_function__PassiveObservation__y_observation(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__get_function__PassiveObservation__y_observation(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__resize_function__PassiveObservation__y_observation(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__size_function__PassiveObservation__y_shape(
  const void * untyped_member)
{
  const rosidl_runtime_c__int64__Sequence * member =
    (const rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return member->size;
}

const void * interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__get_const_function__PassiveObservation__y_shape(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int64__Sequence * member =
    (const rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return &member->data[index];
}

void * interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__get_function__PassiveObservation__y_shape(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int64__Sequence * member =
    (rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return &member->data[index];
}

void interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__fetch_function__PassiveObservation__y_shape(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int64_t * item =
    ((const int64_t *)
    interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__get_const_function__PassiveObservation__y_shape(untyped_member, index));
  int64_t * value =
    (int64_t *)(untyped_value);
  *value = *item;
}

void interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__assign_function__PassiveObservation__y_shape(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int64_t * item =
    ((int64_t *)
    interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__get_function__PassiveObservation__y_shape(untyped_member, index));
  const int64_t * value =
    (const int64_t *)(untyped_value);
  *item = *value;
}

bool interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__resize_function__PassiveObservation__y_shape(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int64__Sequence * member =
    (rosidl_runtime_c__int64__Sequence *)(untyped_member);
  rosidl_runtime_c__int64__Sequence__fini(member);
  return rosidl_runtime_c__int64__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__PassiveObservation_message_member_array[2] = {
  {
    "y_observation",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces__msg__PassiveObservation, y_observation),  // bytes offset in struct
    NULL,  // default value
    interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__size_function__PassiveObservation__y_observation,  // size() function pointer
    interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__get_const_function__PassiveObservation__y_observation,  // get_const(index) function pointer
    interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__get_function__PassiveObservation__y_observation,  // get(index) function pointer
    interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__fetch_function__PassiveObservation__y_observation,  // fetch(index, &value) function pointer
    interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__assign_function__PassiveObservation__y_observation,  // assign(index, value) function pointer
    interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__resize_function__PassiveObservation__y_observation  // resize(index) function pointer
  },
  {
    "y_shape",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces__msg__PassiveObservation, y_shape),  // bytes offset in struct
    NULL,  // default value
    interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__size_function__PassiveObservation__y_shape,  // size() function pointer
    interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__get_const_function__PassiveObservation__y_shape,  // get_const(index) function pointer
    interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__get_function__PassiveObservation__y_shape,  // get(index) function pointer
    interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__fetch_function__PassiveObservation__y_shape,  // fetch(index, &value) function pointer
    interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__assign_function__PassiveObservation__y_shape,  // assign(index, value) function pointer
    interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__resize_function__PassiveObservation__y_shape  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__PassiveObservation_message_members = {
  "interfaces__msg",  // message namespace
  "PassiveObservation",  // message name
  2,  // number of fields
  sizeof(interfaces__msg__PassiveObservation),
  interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__PassiveObservation_message_member_array,  // message members
  interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__PassiveObservation_init_function,  // function to initialize message memory (memory has to be allocated)
  interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__PassiveObservation_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__PassiveObservation_message_type_support_handle = {
  0,
  &interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__PassiveObservation_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interfaces, msg, PassiveObservation)() {
  if (!interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__PassiveObservation_message_type_support_handle.typesupport_identifier) {
    interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__PassiveObservation_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &interfaces__msg__PassiveObservation__rosidl_typesupport_introspection_c__PassiveObservation_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
