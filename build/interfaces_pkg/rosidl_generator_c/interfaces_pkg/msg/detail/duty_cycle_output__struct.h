// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces_pkg:msg/DutyCycleOutput.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES_PKG__MSG__DETAIL__DUTY_CYCLE_OUTPUT__STRUCT_H_
#define INTERFACES_PKG__MSG__DETAIL__DUTY_CYCLE_OUTPUT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/DutyCycleOutput in the package interfaces_pkg.
typedef struct interfaces_pkg__msg__DutyCycleOutput
{
  double left_drive_output;
  double right_drive_output;
  double left_lift_output;
  double right_lift_output;
  double tilt_output;
  double vibrator_output;
} interfaces_pkg__msg__DutyCycleOutput;

// Struct for a sequence of interfaces_pkg__msg__DutyCycleOutput.
typedef struct interfaces_pkg__msg__DutyCycleOutput__Sequence
{
  interfaces_pkg__msg__DutyCycleOutput * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces_pkg__msg__DutyCycleOutput__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES_PKG__MSG__DETAIL__DUTY_CYCLE_OUTPUT__STRUCT_H_
