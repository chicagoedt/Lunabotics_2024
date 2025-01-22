// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interfaces_pkg:msg/DutyCycleOutput.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES_PKG__MSG__DETAIL__DUTY_CYCLE_OUTPUT__TRAITS_HPP_
#define INTERFACES_PKG__MSG__DETAIL__DUTY_CYCLE_OUTPUT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interfaces_pkg/msg/detail/duty_cycle_output__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interfaces_pkg
{

namespace msg
{

inline void to_flow_style_yaml(
  const DutyCycleOutput & msg,
  std::ostream & out)
{
  out << "{";
  // member: left_drive_output
  {
    out << "left_drive_output: ";
    rosidl_generator_traits::value_to_yaml(msg.left_drive_output, out);
    out << ", ";
  }

  // member: right_drive_output
  {
    out << "right_drive_output: ";
    rosidl_generator_traits::value_to_yaml(msg.right_drive_output, out);
    out << ", ";
  }

  // member: left_lift_output
  {
    out << "left_lift_output: ";
    rosidl_generator_traits::value_to_yaml(msg.left_lift_output, out);
    out << ", ";
  }

  // member: right_lift_output
  {
    out << "right_lift_output: ";
    rosidl_generator_traits::value_to_yaml(msg.right_lift_output, out);
    out << ", ";
  }

  // member: tilt_output
  {
    out << "tilt_output: ";
    rosidl_generator_traits::value_to_yaml(msg.tilt_output, out);
    out << ", ";
  }

  // member: vibrator_output
  {
    out << "vibrator_output: ";
    rosidl_generator_traits::value_to_yaml(msg.vibrator_output, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DutyCycleOutput & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: left_drive_output
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_drive_output: ";
    rosidl_generator_traits::value_to_yaml(msg.left_drive_output, out);
    out << "\n";
  }

  // member: right_drive_output
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_drive_output: ";
    rosidl_generator_traits::value_to_yaml(msg.right_drive_output, out);
    out << "\n";
  }

  // member: left_lift_output
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_lift_output: ";
    rosidl_generator_traits::value_to_yaml(msg.left_lift_output, out);
    out << "\n";
  }

  // member: right_lift_output
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_lift_output: ";
    rosidl_generator_traits::value_to_yaml(msg.right_lift_output, out);
    out << "\n";
  }

  // member: tilt_output
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tilt_output: ";
    rosidl_generator_traits::value_to_yaml(msg.tilt_output, out);
    out << "\n";
  }

  // member: vibrator_output
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vibrator_output: ";
    rosidl_generator_traits::value_to_yaml(msg.vibrator_output, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DutyCycleOutput & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace interfaces_pkg

namespace rosidl_generator_traits
{

[[deprecated("use interfaces_pkg::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interfaces_pkg::msg::DutyCycleOutput & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces_pkg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces_pkg::msg::to_yaml() instead")]]
inline std::string to_yaml(const interfaces_pkg::msg::DutyCycleOutput & msg)
{
  return interfaces_pkg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces_pkg::msg::DutyCycleOutput>()
{
  return "interfaces_pkg::msg::DutyCycleOutput";
}

template<>
inline const char * name<interfaces_pkg::msg::DutyCycleOutput>()
{
  return "interfaces_pkg/msg/DutyCycleOutput";
}

template<>
struct has_fixed_size<interfaces_pkg::msg::DutyCycleOutput>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<interfaces_pkg::msg::DutyCycleOutput>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<interfaces_pkg::msg::DutyCycleOutput>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACES_PKG__MSG__DETAIL__DUTY_CYCLE_OUTPUT__TRAITS_HPP_
