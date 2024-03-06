#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "robot_model::robot_model" for configuration "Release"
set_property(TARGET robot_model::robot_model APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(robot_model::robot_model PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/librobot_model.so"
  IMPORTED_SONAME_RELEASE "librobot_model.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS robot_model::robot_model )
list(APPEND _IMPORT_CHECK_FILES_FOR_robot_model::robot_model "${_IMPORT_PREFIX}/lib/librobot_model.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)