// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from interfaces_pkg:msg/DutyCycleOutput.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "interfaces_pkg/msg/detail/duty_cycle_output__rosidl_typesupport_introspection_c.h"
#include "interfaces_pkg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "interfaces_pkg/msg/detail/duty_cycle_output__functions.h"
#include "interfaces_pkg/msg/detail/duty_cycle_output__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void interfaces_pkg__msg__DutyCycleOutput__rosidl_typesupport_introspection_c__DutyCycleOutput_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  interfaces_pkg__msg__DutyCycleOutput__init(message_memory);
}

void interfaces_pkg__msg__DutyCycleOutput__rosidl_typesupport_introspection_c__DutyCycleOutput_fini_function(void * message_memory)
{
  interfaces_pkg__msg__DutyCycleOutput__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember interfaces_pkg__msg__DutyCycleOutput__rosidl_typesupport_introspection_c__DutyCycleOutput_message_member_array[6] = {
  {
    "left_drive_output",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces_pkg__msg__DutyCycleOutput, left_drive_output),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "right_drive_output",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces_pkg__msg__DutyCycleOutput, right_drive_output),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "left_lift_output",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces_pkg__msg__DutyCycleOutput, left_lift_output),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "right_lift_output",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces_pkg__msg__DutyCycleOutput, right_lift_output),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tilt_output",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces_pkg__msg__DutyCycleOutput, tilt_output),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "vibrator_output",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces_pkg__msg__DutyCycleOutput, vibrator_output),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers interfaces_pkg__msg__DutyCycleOutput__rosidl_typesupport_introspection_c__DutyCycleOutput_message_members = {
  "interfaces_pkg__msg",  // message namespace
  "DutyCycleOutput",  // message name
  6,  // number of fields
  sizeof(interfaces_pkg__msg__DutyCycleOutput),
  interfaces_pkg__msg__DutyCycleOutput__rosidl_typesupport_introspection_c__DutyCycleOutput_message_member_array,  // message members
  interfaces_pkg__msg__DutyCycleOutput__rosidl_typesupport_introspection_c__DutyCycleOutput_init_function,  // function to initialize message memory (memory has to be allocated)
  interfaces_pkg__msg__DutyCycleOutput__rosidl_typesupport_introspection_c__DutyCycleOutput_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t interfaces_pkg__msg__DutyCycleOutput__rosidl_typesupport_introspection_c__DutyCycleOutput_message_type_support_handle = {
  0,
  &interfaces_pkg__msg__DutyCycleOutput__rosidl_typesupport_introspection_c__DutyCycleOutput_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_interfaces_pkg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interfaces_pkg, msg, DutyCycleOutput)() {
  if (!interfaces_pkg__msg__DutyCycleOutput__rosidl_typesupport_introspection_c__DutyCycleOutput_message_type_support_handle.typesupport_identifier) {
    interfaces_pkg__msg__DutyCycleOutput__rosidl_typesupport_introspection_c__DutyCycleOutput_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &interfaces_pkg__msg__DutyCycleOutput__rosidl_typesupport_introspection_c__DutyCycleOutput_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
