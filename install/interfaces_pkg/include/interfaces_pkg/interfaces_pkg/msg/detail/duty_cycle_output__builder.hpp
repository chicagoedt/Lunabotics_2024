// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces_pkg:msg/DutyCycleOutput.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES_PKG__MSG__DETAIL__DUTY_CYCLE_OUTPUT__BUILDER_HPP_
#define INTERFACES_PKG__MSG__DETAIL__DUTY_CYCLE_OUTPUT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces_pkg/msg/detail/duty_cycle_output__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces_pkg
{

namespace msg
{

namespace builder
{

class Init_DutyCycleOutput_vibrator_output
{
public:
  explicit Init_DutyCycleOutput_vibrator_output(::interfaces_pkg::msg::DutyCycleOutput & msg)
  : msg_(msg)
  {}
  ::interfaces_pkg::msg::DutyCycleOutput vibrator_output(::interfaces_pkg::msg::DutyCycleOutput::_vibrator_output_type arg)
  {
    msg_.vibrator_output = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces_pkg::msg::DutyCycleOutput msg_;
};

class Init_DutyCycleOutput_tilt_output
{
public:
  explicit Init_DutyCycleOutput_tilt_output(::interfaces_pkg::msg::DutyCycleOutput & msg)
  : msg_(msg)
  {}
  Init_DutyCycleOutput_vibrator_output tilt_output(::interfaces_pkg::msg::DutyCycleOutput::_tilt_output_type arg)
  {
    msg_.tilt_output = std::move(arg);
    return Init_DutyCycleOutput_vibrator_output(msg_);
  }

private:
  ::interfaces_pkg::msg::DutyCycleOutput msg_;
};

class Init_DutyCycleOutput_right_lift_output
{
public:
  explicit Init_DutyCycleOutput_right_lift_output(::interfaces_pkg::msg::DutyCycleOutput & msg)
  : msg_(msg)
  {}
  Init_DutyCycleOutput_tilt_output right_lift_output(::interfaces_pkg::msg::DutyCycleOutput::_right_lift_output_type arg)
  {
    msg_.right_lift_output = std::move(arg);
    return Init_DutyCycleOutput_tilt_output(msg_);
  }

private:
  ::interfaces_pkg::msg::DutyCycleOutput msg_;
};

class Init_DutyCycleOutput_left_lift_output
{
public:
  explicit Init_DutyCycleOutput_left_lift_output(::interfaces_pkg::msg::DutyCycleOutput & msg)
  : msg_(msg)
  {}
  Init_DutyCycleOutput_right_lift_output left_lift_output(::interfaces_pkg::msg::DutyCycleOutput::_left_lift_output_type arg)
  {
    msg_.left_lift_output = std::move(arg);
    return Init_DutyCycleOutput_right_lift_output(msg_);
  }

private:
  ::interfaces_pkg::msg::DutyCycleOutput msg_;
};

class Init_DutyCycleOutput_right_drive_output
{
public:
  explicit Init_DutyCycleOutput_right_drive_output(::interfaces_pkg::msg::DutyCycleOutput & msg)
  : msg_(msg)
  {}
  Init_DutyCycleOutput_left_lift_output right_drive_output(::interfaces_pkg::msg::DutyCycleOutput::_right_drive_output_type arg)
  {
    msg_.right_drive_output = std::move(arg);
    return Init_DutyCycleOutput_left_lift_output(msg_);
  }

private:
  ::interfaces_pkg::msg::DutyCycleOutput msg_;
};

class Init_DutyCycleOutput_left_drive_output
{
public:
  Init_DutyCycleOutput_left_drive_output()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DutyCycleOutput_right_drive_output left_drive_output(::interfaces_pkg::msg::DutyCycleOutput::_left_drive_output_type arg)
  {
    msg_.left_drive_output = std::move(arg);
    return Init_DutyCycleOutput_right_drive_output(msg_);
  }

private:
  ::interfaces_pkg::msg::DutyCycleOutput msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces_pkg::msg::DutyCycleOutput>()
{
  return interfaces_pkg::msg::builder::Init_DutyCycleOutput_left_drive_output();
}

}  // namespace interfaces_pkg

#endif  // INTERFACES_PKG__MSG__DETAIL__DUTY_CYCLE_OUTPUT__BUILDER_HPP_
