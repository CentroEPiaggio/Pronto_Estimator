# Generated by CMake

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 2.6)
   message(FATAL_ERROR "CMake >= 2.6.0 required")
endif()
cmake_policy(PUSH)
cmake_policy(VERSION 2.6...3.20)
#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Protect against multiple inclusion, which would fail when already imported targets are added once more.
set(_targetsDefined)
set(_targetsNotDefined)
set(_expectedTargets)
foreach(_expectedTarget rviz_legged_plugins::rviz_legged_plugins)
  list(APPEND _expectedTargets ${_expectedTarget})
  if(NOT TARGET ${_expectedTarget})
    list(APPEND _targetsNotDefined ${_expectedTarget})
  endif()
  if(TARGET ${_expectedTarget})
    list(APPEND _targetsDefined ${_expectedTarget})
  endif()
endforeach()
if("${_targetsDefined}" STREQUAL "${_expectedTargets}")
  unset(_targetsDefined)
  unset(_targetsNotDefined)
  unset(_expectedTargets)
  set(CMAKE_IMPORT_FILE_VERSION)
  cmake_policy(POP)
  return()
endif()
if(NOT "${_targetsDefined}" STREQUAL "")
  message(FATAL_ERROR "Some (but not all) targets in this export set were already defined.\nTargets Defined: ${_targetsDefined}\nTargets not yet defined: ${_targetsNotDefined}\n")
endif()
unset(_targetsDefined)
unset(_targetsNotDefined)
unset(_expectedTargets)


# Compute the installation prefix relative to this file.
get_filename_component(_IMPORT_PREFIX "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)
get_filename_component(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)
get_filename_component(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)
if(_IMPORT_PREFIX STREQUAL "/")
  set(_IMPORT_PREFIX "")
endif()

# Create imported target rviz_legged_plugins::rviz_legged_plugins
add_library(rviz_legged_plugins::rviz_legged_plugins SHARED IMPORTED)

set_target_properties(rviz_legged_plugins::rviz_legged_plugins PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include/rviz_legged_plugins;/usr/include/x86_64-linux-gnu/qt5/;/usr/include/x86_64-linux-gnu/qt5/QtWidgets;/usr/include/x86_64-linux-gnu/qt5/QtGui;/usr/include/x86_64-linux-gnu/qt5/QtCore;/usr/lib/x86_64-linux-gnu/qt5//mkspecs/linux-g++"
  INTERFACE_LINK_LIBRARIES "rviz_ogre_vendor::OgreMain;rviz_ogre_vendor::OgreOverlay;rviz_legged_msgs::rviz_legged_msgs__rosidl_generator_c;rviz_legged_msgs::rviz_legged_msgs__rosidl_typesupport_fastrtps_c;rviz_legged_msgs::rviz_legged_msgs__rosidl_typesupport_introspection_c;rviz_legged_msgs::rviz_legged_msgs__rosidl_typesupport_c;rviz_legged_msgs::rviz_legged_msgs__rosidl_generator_cpp;rviz_legged_msgs::rviz_legged_msgs__rosidl_typesupport_fastrtps_cpp;rviz_legged_msgs::rviz_legged_msgs__rosidl_typesupport_introspection_cpp;rviz_legged_msgs::rviz_legged_msgs__rosidl_typesupport_cpp;rviz_legged_msgs::rviz_legged_msgs__rosidl_generator_py;image_transport::image_transport;interactive_markers::interactive_markers;laser_geometry::laser_geometry;map_msgs::map_msgs__rosidl_generator_c;map_msgs::map_msgs__rosidl_typesupport_fastrtps_c;map_msgs::map_msgs__rosidl_typesupport_introspection_c;map_msgs::map_msgs__rosidl_typesupport_c;map_msgs::map_msgs__rosidl_generator_cpp;map_msgs::map_msgs__rosidl_typesupport_fastrtps_cpp;map_msgs::map_msgs__rosidl_typesupport_introspection_cpp;map_msgs::map_msgs__rosidl_typesupport_cpp;map_msgs::map_msgs__rosidl_generator_py;nav_msgs::nav_msgs__rosidl_generator_c;nav_msgs::nav_msgs__rosidl_typesupport_fastrtps_c;nav_msgs::nav_msgs__rosidl_generator_cpp;nav_msgs::nav_msgs__rosidl_typesupport_fastrtps_cpp;nav_msgs::nav_msgs__rosidl_typesupport_introspection_c;nav_msgs::nav_msgs__rosidl_typesupport_c;nav_msgs::nav_msgs__rosidl_typesupport_introspection_cpp;nav_msgs::nav_msgs__rosidl_typesupport_cpp;nav_msgs::nav_msgs__rosidl_generator_py;rclcpp::rclcpp;resource_retriever::resource_retriever;rviz_common::rviz_common;rviz_rendering::rviz_rendering;sensor_msgs::sensor_msgs__rosidl_generator_c;sensor_msgs::sensor_msgs__rosidl_typesupport_fastrtps_c;sensor_msgs::sensor_msgs__rosidl_generator_cpp;sensor_msgs::sensor_msgs__rosidl_typesupport_fastrtps_cpp;sensor_msgs::sensor_msgs__rosidl_typesupport_introspection_c;sensor_msgs::sensor_msgs__rosidl_typesupport_c;sensor_msgs::sensor_msgs__rosidl_typesupport_introspection_cpp;sensor_msgs::sensor_msgs__rosidl_typesupport_cpp;sensor_msgs::sensor_msgs__rosidl_generator_py;sensor_msgs::sensor_msgs_library;tf2::tf2;tf2_geometry_msgs::tf2_geometry_msgs;tf2_ros::tf2_ros;tf2_ros::static_transform_broadcaster_node;urdf::urdf;visualization_msgs::visualization_msgs__rosidl_generator_c;visualization_msgs::visualization_msgs__rosidl_typesupport_fastrtps_c;visualization_msgs::visualization_msgs__rosidl_generator_cpp;visualization_msgs::visualization_msgs__rosidl_typesupport_fastrtps_cpp;visualization_msgs::visualization_msgs__rosidl_typesupport_introspection_c;visualization_msgs::visualization_msgs__rosidl_typesupport_c;visualization_msgs::visualization_msgs__rosidl_typesupport_introspection_cpp;visualization_msgs::visualization_msgs__rosidl_typesupport_cpp;visualization_msgs::visualization_msgs__rosidl_generator_py;rviz_default_plugins::rviz_default_plugins"
)

if(CMAKE_VERSION VERSION_LESS 2.8.12)
  message(FATAL_ERROR "This file relies on consumers using CMake 2.8.12 or greater.")
endif()

# Load information for each installed configuration.
get_filename_component(_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
file(GLOB CONFIG_FILES "${_DIR}/rviz_legged_pluginsExport-*.cmake")
foreach(f ${CONFIG_FILES})
  include(${f})
endforeach()

# Cleanup temporary variables.
set(_IMPORT_PREFIX)

# Loop over all imported files and verify that they actually exist
foreach(target ${_IMPORT_CHECK_TARGETS} )
  foreach(file ${_IMPORT_CHECK_FILES_FOR_${target}} )
    if(NOT EXISTS "${file}" )
      message(FATAL_ERROR "The imported target \"${target}\" references the file
   \"${file}\"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   \"${CMAKE_CURRENT_LIST_FILE}\"
but not all the files it references.
")
    endif()
  endforeach()
  unset(_IMPORT_CHECK_FILES_FOR_${target})
endforeach()
unset(_IMPORT_CHECK_TARGETS)

# This file does not depend on other imported targets which have
# been exported from the same project but in a separate export set.

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
cmake_policy(POP)
