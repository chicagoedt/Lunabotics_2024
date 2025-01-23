// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces_pkg:msg/DutyCycleOutput.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES_PKG__MSG__DETAIL__DUTY_CYCLE_OUTPUT__STRUCT_HPP_
#define INTERFACES_PKG__MSG__DETAIL__DUTY_CYCLE_OUTPUT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces_pkg__msg__DutyCycleOutput __attribute__((deprecated))
#else
# define DEPRECATED__interfaces_pkg__msg__DutyCycleOutput __declspec(deprecated)
#endif

namespace interfaces_pkg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DutyCycleOutput_
{
  using Type = DutyCycleOutput_<ContainerAllocator>;

  explicit DutyCycleOutput_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left_drive_output = 0.0;
      this->right_drive_output = 0.0;
      this->left_lift_output = 0.0;
      this->right_lift_output = 0.0;
      this->tilt_output = 0.0;
      this->vibrator_output = 0.0;
    }
  }

  explicit DutyCycleOutput_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left_drive_output = 0.0;
      this->right_drive_output = 0.0;
      this->left_lift_output = 0.0;
      this->right_lift_output = 0.0;
      this->tilt_output = 0.0;
      this->vibrator_output = 0.0;
    }
  }

  // field types and members
  using _left_drive_output_type =
    double;
  _left_drive_output_type left_drive_output;
  using _right_drive_output_type =
    double;
  _right_drive_output_type right_drive_output;
  using _left_lift_output_type =
    double;
  _left_lift_output_type left_lift_output;
  using _right_lift_output_type =
    double;
  _right_lift_output_type right_lift_output;
  using _tilt_output_type =
    double;
  _tilt_output_type tilt_output;
  using _vibrator_output_type =
    double;
  _vibrator_output_type vibrator_output;

  // setters for named parameter idiom
  Type & set__left_drive_output(
    const double & _arg)
  {
    this->left_drive_output = _arg;
    return *this;
  }
  Type & set__right_drive_output(
    const double & _arg)
  {
    this->right_drive_output = _arg;
    return *this;
  }
  Type & set__left_lift_output(
    const double & _arg)
  {
    this->left_lift_output = _arg;
    return *this;
  }
  Type & set__right_lift_output(
    const double & _arg)
  {
    this->right_lift_output = _arg;
    return *this;
  }
  Type & set__tilt_output(
    const double & _arg)
  {
    this->tilt_output = _arg;
    return *this;
  }
  Type & set__vibrator_output(
    const double & _arg)
  {
    this->vibrator_output = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces_pkg::msg::DutyCycleOutput_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces_pkg::msg::DutyCycleOutput_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces_pkg::msg::DutyCycleOutput_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces_pkg::msg::DutyCycleOutput_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces_pkg::msg::DutyCycleOutput_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces_pkg::msg::DutyCycleOutput_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces_pkg::msg::DutyCycleOutput_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces_pkg::msg::DutyCycleOutput_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces_pkg::msg::DutyCycleOutput_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces_pkg::msg::DutyCycleOutput_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces_pkg__msg__DutyCycleOutput
    std::shared_ptr<interfaces_pkg::msg::DutyCycleOutput_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces_pkg__msg__DutyCycleOutput
    std::shared_ptr<interfaces_pkg::msg::DutyCycleOutput_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DutyCycleOutput_ & other) const
  {
    if (this->left_drive_output != other.left_drive_output) {
      return false;
    }
    if (this->right_drive_output != other.right_drive_output) {
      return false;
    }
    if (this->left_lift_output != other.left_lift_output) {
      return false;
    }
    if (this->right_lift_output != other.right_lift_output) {
      return false;
    }
    if (this->tilt_output != other.tilt_output) {
      return false;
    }
    if (this->vibrator_output != other.vibrator_output) {
      return false;
    }
    return true;
  }
  bool operator!=(const DutyCycleOutput_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DutyCycleOutput_

// alias to use template instance with default allocator
using DutyCycleOutput =
  interfaces_pkg::msg::DutyCycleOutput_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces_pkg

#endif  // INTERFACES_PKG__MSG__DETAIL__DUTY_CYCLE_OUTPUT__STRUCT_HPP_
