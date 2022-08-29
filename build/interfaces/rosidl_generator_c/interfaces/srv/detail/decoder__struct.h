// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:srv/Decoder.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__DECODER__STRUCT_H_
#define INTERFACES__SRV__DETAIL__DECODER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Decoder in the package interfaces.
typedef struct interfaces__srv__Decoder_Request
{
  bool req;
} interfaces__srv__Decoder_Request;

// Struct for a sequence of interfaces__srv__Decoder_Request.
typedef struct interfaces__srv__Decoder_Request__Sequence
{
  interfaces__srv__Decoder_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__srv__Decoder_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'res'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/Decoder in the package interfaces.
typedef struct interfaces__srv__Decoder_Response
{
  rosidl_runtime_c__double__Sequence res;
} interfaces__srv__Decoder_Response;

// Struct for a sequence of interfaces__srv__Decoder_Response.
typedef struct interfaces__srv__Decoder_Response__Sequence
{
  interfaces__srv__Decoder_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__srv__Decoder_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__SRV__DETAIL__DECODER__STRUCT_H_
