// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interfaces:msg/Sample.idl
// generated code does not contain a copyright notice
#include "interfaces/msg/detail/sample__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `x_state`
// Member `y_observation`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
interfaces__msg__Sample__init(interfaces__msg__Sample * msg)
{
  if (!msg) {
    return false;
  }
  // x_state
  if (!rosidl_runtime_c__double__Sequence__init(&msg->x_state, 0)) {
    interfaces__msg__Sample__fini(msg);
    return false;
  }
  // y_observation
  if (!rosidl_runtime_c__double__Sequence__init(&msg->y_observation, 0)) {
    interfaces__msg__Sample__fini(msg);
    return false;
  }
  return true;
}

void
interfaces__msg__Sample__fini(interfaces__msg__Sample * msg)
{
  if (!msg) {
    return;
  }
  // x_state
  rosidl_runtime_c__double__Sequence__fini(&msg->x_state);
  // y_observation
  rosidl_runtime_c__double__Sequence__fini(&msg->y_observation);
}

bool
interfaces__msg__Sample__are_equal(const interfaces__msg__Sample * lhs, const interfaces__msg__Sample * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x_state
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->x_state), &(rhs->x_state)))
  {
    return false;
  }
  // y_observation
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->y_observation), &(rhs->y_observation)))
  {
    return false;
  }
  return true;
}

bool
interfaces__msg__Sample__copy(
  const interfaces__msg__Sample * input,
  interfaces__msg__Sample * output)
{
  if (!input || !output) {
    return false;
  }
  // x_state
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->x_state), &(output->x_state)))
  {
    return false;
  }
  // y_observation
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->y_observation), &(output->y_observation)))
  {
    return false;
  }
  return true;
}

interfaces__msg__Sample *
interfaces__msg__Sample__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__Sample * msg = (interfaces__msg__Sample *)allocator.allocate(sizeof(interfaces__msg__Sample), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interfaces__msg__Sample));
  bool success = interfaces__msg__Sample__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interfaces__msg__Sample__destroy(interfaces__msg__Sample * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interfaces__msg__Sample__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interfaces__msg__Sample__Sequence__init(interfaces__msg__Sample__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__Sample * data = NULL;

  if (size) {
    data = (interfaces__msg__Sample *)allocator.zero_allocate(size, sizeof(interfaces__msg__Sample), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interfaces__msg__Sample__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interfaces__msg__Sample__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
interfaces__msg__Sample__Sequence__fini(interfaces__msg__Sample__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      interfaces__msg__Sample__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

interfaces__msg__Sample__Sequence *
interfaces__msg__Sample__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__Sample__Sequence * array = (interfaces__msg__Sample__Sequence *)allocator.allocate(sizeof(interfaces__msg__Sample__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interfaces__msg__Sample__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interfaces__msg__Sample__Sequence__destroy(interfaces__msg__Sample__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interfaces__msg__Sample__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interfaces__msg__Sample__Sequence__are_equal(const interfaces__msg__Sample__Sequence * lhs, const interfaces__msg__Sample__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interfaces__msg__Sample__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interfaces__msg__Sample__Sequence__copy(
  const interfaces__msg__Sample__Sequence * input,
  interfaces__msg__Sample__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interfaces__msg__Sample);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    interfaces__msg__Sample * data =
      (interfaces__msg__Sample *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interfaces__msg__Sample__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          interfaces__msg__Sample__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interfaces__msg__Sample__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
