# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target rm_energy_interfaces::rm_energy_interfaces
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${rm_energy_interfaces_TARGETS}.
if(rm_energy_interfaces_TARGETS AND NOT TARGET rm_energy_interfaces::rm_energy_interfaces)
  add_library(rm_energy_interfaces::rm_energy_interfaces INTERFACE IMPORTED)
  set_target_properties(rm_energy_interfaces::rm_energy_interfaces PROPERTIES
    INTERFACE_LINK_LIBRARIES "${rm_energy_interfaces_TARGETS}")
endif()
