#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "quadprog::quadprog" for configuration "Release"
set_property(TARGET quadprog::quadprog APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(quadprog::quadprog PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libquadprog.so"
  IMPORTED_SONAME_RELEASE "libquadprog.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS quadprog::quadprog )
list(APPEND _IMPORT_CHECK_FILES_FOR_quadprog::quadprog "${_IMPORT_PREFIX}/lib/libquadprog.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
