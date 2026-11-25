// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rm_energy_interfaces:msg/GimbalCommand.idl
// generated code does not contain a copyright notice

#ifndef RM_ENERGY_INTERFACES__MSG__DETAIL__GIMBAL_COMMAND__STRUCT_H_
#define RM_ENERGY_INTERFACES__MSG__DETAIL__GIMBAL_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/GimbalCommand in the package rm_energy_interfaces.
typedef struct rm_energy_interfaces__msg__GimbalCommand
{
  std_msgs__msg__Header header;
  double yaw_rad;
  double pitch_rad;
  bool fire;
} rm_energy_interfaces__msg__GimbalCommand;

// Struct for a sequence of rm_energy_interfaces__msg__GimbalCommand.
typedef struct rm_energy_interfaces__msg__GimbalCommand__Sequence
{
  rm_energy_interfaces__msg__GimbalCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rm_energy_interfaces__msg__GimbalCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RM_ENERGY_INTERFACES__MSG__DETAIL__GIMBAL_COMMAND__STRUCT_H_
