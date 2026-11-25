// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rm_energy_interfaces:msg/GimbalCommand.idl
// generated code does not contain a copyright notice
#include "rm_energy_interfaces/msg/detail/gimbal_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
rm_energy_interfaces__msg__GimbalCommand__init(rm_energy_interfaces__msg__GimbalCommand * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    rm_energy_interfaces__msg__GimbalCommand__fini(msg);
    return false;
  }
  // yaw_rad
  // pitch_rad
  // fire
  return true;
}

void
rm_energy_interfaces__msg__GimbalCommand__fini(rm_energy_interfaces__msg__GimbalCommand * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // yaw_rad
  // pitch_rad
  // fire
}

bool
rm_energy_interfaces__msg__GimbalCommand__are_equal(const rm_energy_interfaces__msg__GimbalCommand * lhs, const rm_energy_interfaces__msg__GimbalCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // yaw_rad
  if (lhs->yaw_rad != rhs->yaw_rad) {
    return false;
  }
  // pitch_rad
  if (lhs->pitch_rad != rhs->pitch_rad) {
    return false;
  }
  // fire
  if (lhs->fire != rhs->fire) {
    return false;
  }
  return true;
}

bool
rm_energy_interfaces__msg__GimbalCommand__copy(
  const rm_energy_interfaces__msg__GimbalCommand * input,
  rm_energy_interfaces__msg__GimbalCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // yaw_rad
  output->yaw_rad = input->yaw_rad;
  // pitch_rad
  output->pitch_rad = input->pitch_rad;
  // fire
  output->fire = input->fire;
  return true;
}

rm_energy_interfaces__msg__GimbalCommand *
rm_energy_interfaces__msg__GimbalCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rm_energy_interfaces__msg__GimbalCommand * msg = (rm_energy_interfaces__msg__GimbalCommand *)allocator.allocate(sizeof(rm_energy_interfaces__msg__GimbalCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rm_energy_interfaces__msg__GimbalCommand));
  bool success = rm_energy_interfaces__msg__GimbalCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rm_energy_interfaces__msg__GimbalCommand__destroy(rm_energy_interfaces__msg__GimbalCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rm_energy_interfaces__msg__GimbalCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rm_energy_interfaces__msg__GimbalCommand__Sequence__init(rm_energy_interfaces__msg__GimbalCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rm_energy_interfaces__msg__GimbalCommand * data = NULL;

  if (size) {
    data = (rm_energy_interfaces__msg__GimbalCommand *)allocator.zero_allocate(size, sizeof(rm_energy_interfaces__msg__GimbalCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rm_energy_interfaces__msg__GimbalCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rm_energy_interfaces__msg__GimbalCommand__fini(&data[i - 1]);
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
rm_energy_interfaces__msg__GimbalCommand__Sequence__fini(rm_energy_interfaces__msg__GimbalCommand__Sequence * array)
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
      rm_energy_interfaces__msg__GimbalCommand__fini(&array->data[i]);
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

rm_energy_interfaces__msg__GimbalCommand__Sequence *
rm_energy_interfaces__msg__GimbalCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rm_energy_interfaces__msg__GimbalCommand__Sequence * array = (rm_energy_interfaces__msg__GimbalCommand__Sequence *)allocator.allocate(sizeof(rm_energy_interfaces__msg__GimbalCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rm_energy_interfaces__msg__GimbalCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rm_energy_interfaces__msg__GimbalCommand__Sequence__destroy(rm_energy_interfaces__msg__GimbalCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rm_energy_interfaces__msg__GimbalCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rm_energy_interfaces__msg__GimbalCommand__Sequence__are_equal(const rm_energy_interfaces__msg__GimbalCommand__Sequence * lhs, const rm_energy_interfaces__msg__GimbalCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rm_energy_interfaces__msg__GimbalCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rm_energy_interfaces__msg__GimbalCommand__Sequence__copy(
  const rm_energy_interfaces__msg__GimbalCommand__Sequence * input,
  rm_energy_interfaces__msg__GimbalCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rm_energy_interfaces__msg__GimbalCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rm_energy_interfaces__msg__GimbalCommand * data =
      (rm_energy_interfaces__msg__GimbalCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rm_energy_interfaces__msg__GimbalCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rm_energy_interfaces__msg__GimbalCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rm_energy_interfaces__msg__GimbalCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
