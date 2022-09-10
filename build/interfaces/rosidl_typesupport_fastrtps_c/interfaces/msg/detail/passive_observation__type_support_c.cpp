// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from interfaces:msg/PassiveObservation.idl
// generated code does not contain a copyright notice
#include "interfaces/msg/detail/passive_observation__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "interfaces/msg/detail/passive_observation__struct.h"
#include "interfaces/msg/detail/passive_observation__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/primitives_sequence.h"  // y_observation, y_shape
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // y_observation, y_shape

// forward declare type support functions


using _PassiveObservation__ros_msg_type = interfaces__msg__PassiveObservation;

static bool _PassiveObservation__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _PassiveObservation__ros_msg_type * ros_message = static_cast<const _PassiveObservation__ros_msg_type *>(untyped_ros_message);
  // Field name: y_observation
  {
    size_t size = ros_message->y_observation.size;
    auto array_ptr = ros_message->y_observation.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: y_shape
  {
    size_t size = ros_message->y_shape.size;
    auto array_ptr = ros_message->y_shape.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _PassiveObservation__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _PassiveObservation__ros_msg_type * ros_message = static_cast<_PassiveObservation__ros_msg_type *>(untyped_ros_message);
  // Field name: y_observation
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->y_observation.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->y_observation);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->y_observation, size)) {
      fprintf(stderr, "failed to create array for field 'y_observation'");
      return false;
    }
    auto array_ptr = ros_message->y_observation.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: y_shape
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->y_shape.data) {
      rosidl_runtime_c__int64__Sequence__fini(&ros_message->y_shape);
    }
    if (!rosidl_runtime_c__int64__Sequence__init(&ros_message->y_shape, size)) {
      fprintf(stderr, "failed to create array for field 'y_shape'");
      return false;
    }
    auto array_ptr = ros_message->y_shape.data;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interfaces
size_t get_serialized_size_interfaces__msg__PassiveObservation(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PassiveObservation__ros_msg_type * ros_message = static_cast<const _PassiveObservation__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name y_observation
  {
    size_t array_size = ros_message->y_observation.size;
    auto array_ptr = ros_message->y_observation.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name y_shape
  {
    size_t array_size = ros_message->y_shape.size;
    auto array_ptr = ros_message->y_shape.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _PassiveObservation__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_interfaces__msg__PassiveObservation(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interfaces
size_t max_serialized_size_interfaces__msg__PassiveObservation(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: y_observation
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: y_shape
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _PassiveObservation__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_interfaces__msg__PassiveObservation(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_PassiveObservation = {
  "interfaces::msg",
  "PassiveObservation",
  _PassiveObservation__cdr_serialize,
  _PassiveObservation__cdr_deserialize,
  _PassiveObservation__get_serialized_size,
  _PassiveObservation__max_serialized_size
};

static rosidl_message_type_support_t _PassiveObservation__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_PassiveObservation,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, interfaces, msg, PassiveObservation)() {
  return &_PassiveObservation__type_support;
}

#if defined(__cplusplus)
}
#endif
