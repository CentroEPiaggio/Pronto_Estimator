#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "pronto_quadruped_ros::pronto_quadruped_ros" for configuration "Release"
set_property(TARGET pronto_quadruped_ros::pronto_quadruped_ros APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(pronto_quadruped_ros::pronto_quadruped_ros PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libpronto_quadruped_ros.so"
  IMPORTED_SONAME_RELEASE "libpronto_quadruped_ros.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS pronto_quadruped_ros::pronto_quadruped_ros )
list(APPEND _IMPORT_CHECK_FILES_FOR_pronto_quadruped_ros::pronto_quadruped_ros "${_IMPORT_PREFIX}/lib/libpronto_quadruped_ros.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
