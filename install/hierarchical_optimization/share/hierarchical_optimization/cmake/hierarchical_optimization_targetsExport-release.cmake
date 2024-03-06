#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "hierarchical_optimization::hierarchical_optimization" for configuration "Release"
set_property(TARGET hierarchical_optimization::hierarchical_optimization APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(hierarchical_optimization::hierarchical_optimization PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libhierarchical_optimization.so"
  IMPORTED_SONAME_RELEASE "libhierarchical_optimization.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS hierarchical_optimization::hierarchical_optimization )
list(APPEND _IMPORT_CHECK_FILES_FOR_hierarchical_optimization::hierarchical_optimization "${_IMPORT_PREFIX}/lib/libhierarchical_optimization.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
