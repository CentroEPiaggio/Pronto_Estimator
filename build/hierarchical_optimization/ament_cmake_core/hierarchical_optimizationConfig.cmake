# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_hierarchical_optimization_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED hierarchical_optimization_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(hierarchical_optimization_FOUND FALSE)
  elseif(NOT hierarchical_optimization_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(hierarchical_optimization_FOUND FALSE)
  endif()
  return()
endif()
set(_hierarchical_optimization_CONFIG_INCLUDED TRUE)

# output package information
if(NOT hierarchical_optimization_FIND_QUIETLY)
  message(STATUS "Found hierarchical_optimization: 0.0.0 (${hierarchical_optimization_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'hierarchical_optimization' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${hierarchical_optimization_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(hierarchical_optimization_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_targets-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${hierarchical_optimization_DIR}/${_extra}")
endforeach()
