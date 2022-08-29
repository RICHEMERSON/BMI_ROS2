// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/Sample.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__SAMPLE__STRUCT_H_
#define INTERFACES__MSG__DETAIL__SAMPLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'x_state'
// Member 'y_observation'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Sample in the package interfaces.
typedef struct interfaces__msg__Sample
{
  rosidl_runtime_c__double__Sequence x_state;
  rosidl_runtime_c__double__Sequence y_observation;
} interfaces__msg__Sample;

// Struct for a sequence of interfaces__msg__Sample.
typedef struct interfaces__msg__Sample__Sequence
{
  interfaces__msg__Sample * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__Sample__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__SAMPLE__STRUCT_H_
