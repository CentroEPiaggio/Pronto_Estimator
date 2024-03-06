# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ryml_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ryml_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ryml_FOUND FALSE)
  elseif(NOT ryml_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ryml_FOUND FALSE)
  endif()
  return()
endif()
set(_ryml_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ryml_FIND_QUIETLY)
  message(STATUS "Found ryml: 0.0.0 (${ryml_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ryml' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${ryml_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ryml_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ryml_DIR}/${_extra}")
endforeach()
