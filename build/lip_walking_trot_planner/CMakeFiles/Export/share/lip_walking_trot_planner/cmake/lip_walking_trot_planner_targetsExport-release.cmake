#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "lip_walking_trot_planner::lip_walking_trot_planner" for configuration "Release"
set_property(TARGET lip_walking_trot_planner::lip_walking_trot_planner APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(lip_walking_trot_planner::lip_walking_trot_planner PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/liblip_walking_trot_planner.so"
  IMPORTED_SONAME_RELEASE "liblip_walking_trot_planner.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS lip_walking_trot_planner::lip_walking_trot_planner )
list(APPEND _IMPORT_CHECK_FILES_FOR_lip_walking_trot_planner::lip_walking_trot_planner "${_IMPORT_PREFIX}/lib/liblip_walking_trot_planner.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
