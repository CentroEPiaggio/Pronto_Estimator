# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_pronto_tuning_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED pronto_tuning_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(pronto_tuning_FOUND FALSE)
  elseif(NOT pronto_tuning_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(pronto_tuning_FOUND FALSE)
  endif()
  return()
endif()
set(_pronto_tuning_CONFIG_INCLUDED TRUE)

# output package information
if(NOT pronto_tuning_FIND_QUIETLY)
  message(STATUS "Found pronto_tuning: 0.0.0 (${pronto_tuning_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'pronto_tuning' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${pronto_tuning_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(pronto_tuning_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${pronto_tuning_DIR}/${_extra}")
endforeach()
