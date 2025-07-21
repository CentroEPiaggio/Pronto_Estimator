cmake_minimum_required(VERSION 3.8)
project(any_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoolStamped.msg"
  "msg/Event.msg"
  "msg/ExtendedJointState.msg"
  "msg/Float64Stamped.msg"
  "msg/ImuWithTrigger.msg"
  "msg/PointContact.msg"
  "msg/SensorTimeInfo.msg"
  "msg/State.msg"
  "msg/UserInteractionOption.msg"
  "srv/GetString.srv"
  "srv/GetStrings.srv"
  "srv/ProcessFile.srv"
  "srv/SetFloat32.srv"
  "srv/SetFloat64.srv"
  "srv/SetPose.srv"
  "srv/SetString.srv"
  "srv/SetTopic.srv"
  "srv/SetUInt32.srv"
  "srv/Shutdown.srv"
  "srv/Toggle.srv"
  "srv/UserInteraction.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
