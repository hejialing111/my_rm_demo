// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rm_energy_interfaces:msg/GimbalCommand.idl
// generated code does not contain a copyright notice

#ifndef RM_ENERGY_INTERFACES__MSG__DETAIL__GIMBAL_COMMAND__BUILDER_HPP_
#define RM_ENERGY_INTERFACES__MSG__DETAIL__GIMBAL_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rm_energy_interfaces/msg/detail/gimbal_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rm_energy_interfaces
{

namespace msg
{

namespace builder
{

class Init_GimbalCommand_fire
{
public:
  explicit Init_GimbalCommand_fire(::rm_energy_interfaces::msg::GimbalCommand & msg)
  : msg_(msg)
  {}
  ::rm_energy_interfaces::msg::GimbalCommand fire(::rm_energy_interfaces::msg::GimbalCommand::_fire_type arg)
  {
    msg_.fire = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rm_energy_interfaces::msg::GimbalCommand msg_;
};

class Init_GimbalCommand_pitch_rad
{
public:
  explicit Init_GimbalCommand_pitch_rad(::rm_energy_interfaces::msg::GimbalCommand & msg)
  : msg_(msg)
  {}
  Init_GimbalCommand_fire pitch_rad(::rm_energy_interfaces::msg::GimbalCommand::_pitch_rad_type arg)
  {
    msg_.pitch_rad = std::move(arg);
    return Init_GimbalCommand_fire(msg_);
  }

private:
  ::rm_energy_interfaces::msg::GimbalCommand msg_;
};

class Init_GimbalCommand_yaw_rad
{
public:
  explicit Init_GimbalCommand_yaw_rad(::rm_energy_interfaces::msg::GimbalCommand & msg)
  : msg_(msg)
  {}
  Init_GimbalCommand_pitch_rad yaw_rad(::rm_energy_interfaces::msg::GimbalCommand::_yaw_rad_type arg)
  {
    msg_.yaw_rad = std::move(arg);
    return Init_GimbalCommand_pitch_rad(msg_);
  }

private:
  ::rm_energy_interfaces::msg::GimbalCommand msg_;
};

class Init_GimbalCommand_header
{
public:
  Init_GimbalCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GimbalCommand_yaw_rad header(::rm_energy_interfaces::msg::GimbalCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GimbalCommand_yaw_rad(msg_);
  }

private:
  ::rm_energy_interfaces::msg::GimbalCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rm_energy_interfaces::msg::GimbalCommand>()
{
  return rm_energy_interfaces::msg::builder::Init_GimbalCommand_header();
}

}  // namespace rm_energy_interfaces

#endif  // RM_ENERGY_INTERFACES__MSG__DETAIL__GIMBAL_COMMAND__BUILDER_HPP_
