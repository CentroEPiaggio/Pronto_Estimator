#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "teleoperate_robot::teleop_robot_base_node" for configuration "Release"
set_property(TARGET teleoperate_robot::teleop_robot_base_node APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(teleoperate_robot::teleop_robot_base_node PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/teleoperate_robot/teleop_robot_base_node"
  )

list(APPEND _IMPORT_CHECK_TARGETS teleoperate_robot::teleop_robot_base_node )
list(APPEND _IMPORT_CHECK_FILES_FOR_teleoperate_robot::teleop_robot_base_node "${_IMPORT_PREFIX}/lib/teleoperate_robot/teleop_robot_base_node" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
