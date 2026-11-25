// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rm_energy_interfaces:msg/GimbalCommand.idl
// generated code does not contain a copyright notice

#ifndef RM_ENERGY_INTERFACES__MSG__DETAIL__GIMBAL_COMMAND__TRAITS_HPP_
#define RM_ENERGY_INTERFACES__MSG__DETAIL__GIMBAL_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rm_energy_interfaces/msg/detail/gimbal_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rm_energy_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const GimbalCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: yaw_rad
  {
    out << "yaw_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_rad, out);
    out << ", ";
  }

  // member: pitch_rad
  {
    out << "pitch_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_rad, out);
    out << ", ";
  }

  // member: fire
  {
    out << "fire: ";
    rosidl_generator_traits::value_to_yaml(msg.fire, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GimbalCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: yaw_rad
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_rad, out);
    out << "\n";
  }

  // member: pitch_rad
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_rad, out);
    out << "\n";
  }

  // member: fire
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fire: ";
    rosidl_generator_traits::value_to_yaml(msg.fire, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GimbalCommand & msg, bool use_flow_style = false)
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

}  // namespace rm_energy_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use rm_energy_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rm_energy_interfaces::msg::GimbalCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  rm_energy_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rm_energy_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const rm_energy_interfaces::msg::GimbalCommand & msg)
{
  return rm_energy_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rm_energy_interfaces::msg::GimbalCommand>()
{
  return "rm_energy_interfaces::msg::GimbalCommand";
}

template<>
inline const char * name<rm_energy_interfaces::msg::GimbalCommand>()
{
  return "rm_energy_interfaces/msg/GimbalCommand";
}

template<>
struct has_fixed_size<rm_energy_interfaces::msg::GimbalCommand>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<rm_energy_interfaces::msg::GimbalCommand>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<rm_energy_interfaces::msg::GimbalCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RM_ENERGY_INTERFACES__MSG__DETAIL__GIMBAL_COMMAND__TRAITS_HPP_
