cmake_minimum_required(VERSION 3.8)
project(pi3hat_moteus_int_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/PacketPass.msg"
  "msg/JointsStates.msg"
  "msg/JointsCommand.msg"
  "msg/OmniMulinexCommand.msg"
  "msg/QuadFootState.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES 
  std_msgs
  geometry_msgs
  builtin_interfaces
  ADD_LINTER_TESTS
)


ament_export_dependencies(rosidl_default_runtime)
ament_package()
