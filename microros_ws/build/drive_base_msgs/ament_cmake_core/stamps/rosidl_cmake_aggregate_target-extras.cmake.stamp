# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target drive_base_msgs::drive_base_msgs
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${drive_base_msgs_TARGETS}.
if(drive_base_msgs_TARGETS AND NOT TARGET drive_base_msgs::drive_base_msgs)
  add_library(drive_base_msgs::drive_base_msgs INTERFACE IMPORTED)
  set_target_properties(drive_base_msgs::drive_base_msgs PROPERTIES
    INTERFACE_LINK_LIBRARIES "${drive_base_msgs_TARGETS}")
endif()
