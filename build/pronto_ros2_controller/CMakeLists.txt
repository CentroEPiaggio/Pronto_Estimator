cmake_minimum_required(VERSION 3.8)
project(pronto_ros2_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pronto_core REQUIRED)
find_package(pronto_solo12 REQUIRED)
find_package(pronto_quadruped REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(pronto_solo12 REQUIRED)


add_library(
  ${PROJECT_NAME}
  SHARED
  src/pronto_controller.cpp
  src/proprioceptive_sensor_manager.cpp
  src/quad_model_parser.cpp
  src/legodom_manager.cpp
  
)

add_executable(test_node src/test_node.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC

$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>

$<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  builtin_interfaces
  controller_interface
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  rcutils
  pronto_quadruped
  pronto_core
  Eigen3
  pinocchio
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  geometry_msgs
  sensor_msgs
  pronto_solo12
  urdf
  pronto_solo12
)

# ament_target_dependencies(
#   test_node
#   rclcpp
#   urdf
#   ${PROJECT_NAME}
# )

install(
  TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

pluginlib_export_plugin_description_file(controller_interface pronto_controller.xml)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # find_package(urdf REQUIRED)
  # find_package(rclcpp REQUIRED)
  # ament_add_gtest(${PROJECT_NAME}_test test/model_parser_test.cpp)
  # target_include_directories(${PROJECT_NAME}_test PUBLIC
  #     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  #     $<INSTALL_INTERFACE:include>
  #   )

  #   ament_target_dependencies(${PROJECT_NAME}_test
  #   urdf
  #   rclcpp
  # )
  # target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_dependencies(
  builtin_interfaces
  controller_interface
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  rcutils
  pronto_quadruped
  pronto_core
  Eigen3
  pinocchio
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  geometry_msgs
  sensor_msgs
  pronto_solo12
  urdf
  pronto_solo12
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_target_dependencies(
  test_node
  rclcpp
  urdf
  pinocchio
  Eigen3
)

target_link_libraries(test_node ${PROJECT_NAME})
install(TARGETS
test_node
  DESTINATION lib/${PROJECT_NAME})
ament_package()
