// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interfaces_pkg:msg/DutyCycleOutput.idl
// generated code does not contain a copyright notice
#include "interfaces_pkg/msg/detail/duty_cycle_output__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
interfaces_pkg__msg__DutyCycleOutput__init(interfaces_pkg__msg__DutyCycleOutput * msg)
{
  if (!msg) {
    return false;
  }
  // left_drive_output
  // right_drive_output
  // left_lift_output
  // right_lift_output
  // tilt_output
  // vibrator_output
  return true;
}

void
interfaces_pkg__msg__DutyCycleOutput__fini(interfaces_pkg__msg__DutyCycleOutput * msg)
{
  if (!msg) {
    return;
  }
  // left_drive_output
  // right_drive_output
  // left_lift_output
  // right_lift_output
  // tilt_output
  // vibrator_output
}

bool
interfaces_pkg__msg__DutyCycleOutput__are_equal(const interfaces_pkg__msg__DutyCycleOutput * lhs, const interfaces_pkg__msg__DutyCycleOutput * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // left_drive_output
  if (lhs->left_drive_output != rhs->left_drive_output) {
    return false;
  }
  // right_drive_output
  if (lhs->right_drive_output != rhs->right_drive_output) {
    return false;
  }
  // left_lift_output
  if (lhs->left_lift_output != rhs->left_lift_output) {
    return false;
  }
  // right_lift_output
  if (lhs->right_lift_output != rhs->right_lift_output) {
    return false;
  }
  // tilt_output
  if (lhs->tilt_output != rhs->tilt_output) {
    return false;
  }
  // vibrator_output
  if (lhs->vibrator_output != rhs->vibrator_output) {
    return false;
  }
  return true;
}

bool
interfaces_pkg__msg__DutyCycleOutput__copy(
  const interfaces_pkg__msg__DutyCycleOutput * input,
  interfaces_pkg__msg__DutyCycleOutput * output)
{
  if (!input || !output) {
    return false;
  }
  // left_drive_output
  output->left_drive_output = input->left_drive_output;
  // right_drive_output
  output->right_drive_output = input->right_drive_output;
  // left_lift_output
  output->left_lift_output = input->left_lift_output;
  // right_lift_output
  output->right_lift_output = input->right_lift_output;
  // tilt_output
  output->tilt_output = input->tilt_output;
  // vibrator_output
  output->vibrator_output = input->vibrator_output;
  return true;
}

interfaces_pkg__msg__DutyCycleOutput *
interfaces_pkg__msg__DutyCycleOutput__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces_pkg__msg__DutyCycleOutput * msg = (interfaces_pkg__msg__DutyCycleOutput *)allocator.allocate(sizeof(interfaces_pkg__msg__DutyCycleOutput), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interfaces_pkg__msg__DutyCycleOutput));
  bool success = interfaces_pkg__msg__DutyCycleOutput__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interfaces_pkg__msg__DutyCycleOutput__destroy(interfaces_pkg__msg__DutyCycleOutput * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interfaces_pkg__msg__DutyCycleOutput__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interfaces_pkg__msg__DutyCycleOutput__Sequence__init(interfaces_pkg__msg__DutyCycleOutput__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces_pkg__msg__DutyCycleOutput * data = NULL;

  if (size) {
    data = (interfaces_pkg__msg__DutyCycleOutput *)allocator.zero_allocate(size, sizeof(interfaces_pkg__msg__DutyCycleOutput), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interfaces_pkg__msg__DutyCycleOutput__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interfaces_pkg__msg__DutyCycleOutput__fini(&data[i - 1]);
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
interfaces_pkg__msg__DutyCycleOutput__Sequence__fini(interfaces_pkg__msg__DutyCycleOutput__Sequence * array)
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
      interfaces_pkg__msg__DutyCycleOutput__fini(&array->data[i]);
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

interfaces_pkg__msg__DutyCycleOutput__Sequence *
interfaces_pkg__msg__DutyCycleOutput__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces_pkg__msg__DutyCycleOutput__Sequence * array = (interfaces_pkg__msg__DutyCycleOutput__Sequence *)allocator.allocate(sizeof(interfaces_pkg__msg__DutyCycleOutput__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interfaces_pkg__msg__DutyCycleOutput__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interfaces_pkg__msg__DutyCycleOutput__Sequence__destroy(interfaces_pkg__msg__DutyCycleOutput__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interfaces_pkg__msg__DutyCycleOutput__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interfaces_pkg__msg__DutyCycleOutput__Sequence__are_equal(const interfaces_pkg__msg__DutyCycleOutput__Sequence * lhs, const interfaces_pkg__msg__DutyCycleOutput__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interfaces_pkg__msg__DutyCycleOutput__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interfaces_pkg__msg__DutyCycleOutput__Sequence__copy(
  const interfaces_pkg__msg__DutyCycleOutput__Sequence * input,
  interfaces_pkg__msg__DutyCycleOutput__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interfaces_pkg__msg__DutyCycleOutput);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    interfaces_pkg__msg__DutyCycleOutput * data =
      (interfaces_pkg__msg__DutyCycleOutput *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interfaces_pkg__msg__DutyCycleOutput__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          interfaces_pkg__msg__DutyCycleOutput__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interfaces_pkg__msg__DutyCycleOutput__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
