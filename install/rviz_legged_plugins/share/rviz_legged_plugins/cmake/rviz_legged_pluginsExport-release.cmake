#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rviz_legged_plugins::rviz_legged_plugins" for configuration "Release"
set_property(TARGET rviz_legged_plugins::rviz_legged_plugins APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rviz_legged_plugins::rviz_legged_plugins PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/librviz_legged_plugins.so"
  IMPORTED_SONAME_RELEASE "librviz_legged_plugins.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS rviz_legged_plugins::rviz_legged_plugins )
list(APPEND _IMPORT_CHECK_FILES_FOR_rviz_legged_plugins::rviz_legged_plugins "${_IMPORT_PREFIX}/lib/librviz_legged_plugins.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
