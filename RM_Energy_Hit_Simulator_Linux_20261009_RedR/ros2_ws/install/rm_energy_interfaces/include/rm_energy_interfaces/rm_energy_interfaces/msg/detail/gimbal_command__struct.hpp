// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rm_energy_interfaces:msg/GimbalCommand.idl
// generated code does not contain a copyright notice

#ifndef RM_ENERGY_INTERFACES__MSG__DETAIL__GIMBAL_COMMAND__STRUCT_HPP_
#define RM_ENERGY_INTERFACES__MSG__DETAIL__GIMBAL_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rm_energy_interfaces__msg__GimbalCommand __attribute__((deprecated))
#else
# define DEPRECATED__rm_energy_interfaces__msg__GimbalCommand __declspec(deprecated)
#endif

namespace rm_energy_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GimbalCommand_
{
  using Type = GimbalCommand_<ContainerAllocator>;

  explicit GimbalCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->yaw_rad = 0.0;
      this->pitch_rad = 0.0;
      this->fire = false;
    }
  }

  explicit GimbalCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->yaw_rad = 0.0;
      this->pitch_rad = 0.0;
      this->fire = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _yaw_rad_type =
    double;
  _yaw_rad_type yaw_rad;
  using _pitch_rad_type =
    double;
  _pitch_rad_type pitch_rad;
  using _fire_type =
    bool;
  _fire_type fire;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__yaw_rad(
    const double & _arg)
  {
    this->yaw_rad = _arg;
    return *this;
  }
  Type & set__pitch_rad(
    const double & _arg)
  {
    this->pitch_rad = _arg;
    return *this;
  }
  Type & set__fire(
    const bool & _arg)
  {
    this->fire = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rm_energy_interfaces::msg::GimbalCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const rm_energy_interfaces::msg::GimbalCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rm_energy_interfaces::msg::GimbalCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rm_energy_interfaces::msg::GimbalCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rm_energy_interfaces::msg::GimbalCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rm_energy_interfaces::msg::GimbalCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rm_energy_interfaces::msg::GimbalCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rm_energy_interfaces::msg::GimbalCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rm_energy_interfaces::msg::GimbalCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rm_energy_interfaces::msg::GimbalCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rm_energy_interfaces__msg__GimbalCommand
    std::shared_ptr<rm_energy_interfaces::msg::GimbalCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rm_energy_interfaces__msg__GimbalCommand
    std::shared_ptr<rm_energy_interfaces::msg::GimbalCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GimbalCommand_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->yaw_rad != other.yaw_rad) {
      return false;
    }
    if (this->pitch_rad != other.pitch_rad) {
      return false;
    }
    if (this->fire != other.fire) {
      return false;
    }
    return true;
  }
  bool operator!=(const GimbalCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GimbalCommand_

// alias to use template instance with default allocator
using GimbalCommand =
  rm_energy_interfaces::msg::GimbalCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rm_energy_interfaces

#endif  // RM_ENERGY_INTERFACES__MSG__DETAIL__GIMBAL_COMMAND__STRUCT_HPP_
