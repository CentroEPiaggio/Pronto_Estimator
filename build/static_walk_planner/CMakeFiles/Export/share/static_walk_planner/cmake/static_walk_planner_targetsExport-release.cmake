#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "static_walk_planner::static_walk_planner" for configuration "Release"
set_property(TARGET static_walk_planner::static_walk_planner APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(static_walk_planner::static_walk_planner PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libstatic_walk_planner.so"
  IMPORTED_SONAME_RELEASE "libstatic_walk_planner.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS static_walk_planner::static_walk_planner )
list(APPEND _IMPORT_CHECK_FILES_FOR_static_walk_planner::static_walk_planner "${_IMPORT_PREFIX}/lib/libstatic_walk_planner.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
