#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "hqp_controller::hqp_controller" for configuration "Release"
set_property(TARGET hqp_controller::hqp_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(hqp_controller::hqp_controller PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libhqp_controller.so"
  IMPORTED_SONAME_RELEASE "libhqp_controller.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS hqp_controller::hqp_controller )
list(APPEND _IMPORT_CHECK_FILES_FOR_hqp_controller::hqp_controller "${_IMPORT_PREFIX}/lib/libhqp_controller.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
